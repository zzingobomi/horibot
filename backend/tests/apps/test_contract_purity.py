"""contract.py 순수성 불변식 — heavy dep 유출 차단.

계약 규약: contract.py = StrEnum + BaseModel 만 (heavy dep 0). 이 덕에 모든 모듈이
남의 계약을 lazy 고민 없이 top-level import 할 수 있고 (role 격리 유지), payload 는
wire 직렬화 가능한 형태(bytes + 메타 / list[list[float]])로 강제된다. numpy 는 공통
dep 이라 role 격리는 안 깨지만, 계약이 import 하면 ndarray payload 드리프트 신호라
같이 금지.

검증 방식 = AST(import 문 검사) 아님 — 간접 유출(계약이 numpy 를 import 하는 헬퍼를
import)을 못 잡는다. clean subprocess 에서 contract 를 **하나씩 실제로 import** 하며
sys.modules 에 금지 모듈이 새로 나타나는지 검사 → 실제 import 트리 기준이라 우회
불가 + 어느 contract 가 뭘 끌어왔는지 범인 지목. (test_boot 의 role-isolation
subprocess 패턴 동형.)
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]  # backend_v2

# role-split heavy dep + "계약 = 순수 데이터" 위반 신호 (numpy/cv2 등).
_BANNED = (
    "numpy",
    "cv2",
    "open3d",
    "torch",
    "transformers",
    "pyrealsense2",
    "pybullet",
    "ruckig",
    "scipy",
    "fastapi",
    "uvicorn",
    "sqlalchemy",
    "alembic",
    "advanced_alchemy",
    "psutil",
    "zstandard",
)

_PROBE = f"""
import json, sys
from pathlib import Path

BANNED = {_BANNED!r}
names = sorted(p.parent.name for p in Path("modules").glob("*/contract.py"))
assert names, "modules/*/contract.py 발견 0 — glob/cwd 확인"

culprits = {{}}  # banned module -> 처음 끌어온 contract
for name in names:
    __import__(f"modules.{{name}}.contract")
    for banned in BANNED:
        if banned in sys.modules and banned not in culprits:
            culprits[banned] = name

print("PURITY:" + json.dumps({{"contracts": names, "culprits": culprits}}))
"""


def test_contracts_import_no_heavy_deps():
    # clean subprocess — 같은 프로세스는 다른 test 가 이미 heavy dep 을 import 했음.
    r = subprocess.run(
        [sys.executable, "-c", _PROBE], cwd=_ROOT, capture_output=True, text=True
    )
    assert r.returncode == 0, f"probe 실패: stdout={r.stdout} stderr={r.stderr}"

    line = next(ln for ln in r.stdout.splitlines() if ln.startswith("PURITY:"))
    report = json.loads(line[len("PURITY:") :])

    # 전 모듈 계약이 검사 대상에 실제로 들어왔는지 (glob 이 헛돌면 테스트가 무의미)
    assert len(report["contracts"]) >= 10, report["contracts"]

    assert report["culprits"] == {}, (
        "contract.py 가 heavy dep 을 끌어옴 (직접 또는 간접) — 계약은 StrEnum + "
        "BaseModel 만. payload 는 bytes+메타 / list[list[float]] 로: "
        f"{report['culprits']} (banned module → 처음 끌어온 contract)"
    )
