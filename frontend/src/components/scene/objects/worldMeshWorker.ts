/**
 * World 메시 PLY 파싱 워커 — 메인 스레드 히칭 제거 (2026-07-19).
 *
 * 실측: 7.6MB/106k verts PLY 를 PLYLoader 가 110~127ms 에 파싱 — World 갱신
 * (스윕 중 6~7회)마다 메인 스레드가 그만큼 얼어 3D 뷰 프레임 드랍 (사용자
 * 체감 리포트). 파싱을 여기로 옮기고 결과 typed array 는 **transfer** (복사 0)
 * 로 돌려준다 — 메인 스레드에 남는 건 BufferGeometry 조립 + GPU 업로드뿐.
 *
 * 용량 압축(Draco/meshopt)과 별개의 축: 압축은 바이트를 줄일 뿐 파싱/디코드가
 * 여전히 메인에 있으면 히칭은 남는다 — 스레드 이동이 구조적 수정.
 */
import { PLYLoader } from "three/examples/jsm/loaders/PLYLoader.js";

export type WorldMeshRequest = { seq: number; buffer: ArrayBuffer };
export type WorldMeshResult = {
  seq: number;
  parseMs: number;
  position: Float32Array;
  normal: Float32Array | null;
  color: Float32Array | null;
  index: Uint32Array | Uint16Array | null;
};

const ctx = self as unknown as {
  onmessage: ((e: MessageEvent<WorldMeshRequest>) => void) | null;
  postMessage: (msg: WorldMeshResult, transfer: Transferable[]) => void;
};

ctx.onmessage = (e: MessageEvent<WorldMeshRequest>) => {
  const { seq, buffer } = e.data;
  const t0 = performance.now();
  const g = new PLYLoader().parse(buffer);
  if (!g.getAttribute("normal")) g.computeVertexNormals();
  const parseMs = performance.now() - t0;

  const position = g.getAttribute("position").array as Float32Array;
  const normal = (g.getAttribute("normal")?.array as Float32Array) ?? null;
  const color = (g.getAttribute("color")?.array as Float32Array) ?? null;
  const index = (g.getIndex()?.array as Uint32Array | Uint16Array) ?? null;

  // transfer 목록 — 같은 buffer 를 두 번 넣으면 DataCloneError (dedupe).
  const transfer: Transferable[] = [];
  for (const arr of [position, normal, color, index]) {
    if (arr && !transfer.includes(arr.buffer as ArrayBuffer)) {
      transfer.push(arr.buffer as ArrayBuffer);
    }
  }
  ctx.postMessage({ seq, parseMs, position, normal, color, index }, transfer);
};
