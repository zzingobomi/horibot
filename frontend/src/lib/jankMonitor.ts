/**
 * dev 전용 long-frame 계측 — rAF 간격이 문턱을 넘으면 콘솔에 `[jank]`.
 *
 * 용도 = 버벅임의 시각 상관 (2026-07-19 스윕 중 3D 히칭 추적): `[jank]` 가
 * `[world]` 메시 갱신 로그 직후에 몰리면 범인은 메시 갱신 경로(디코드/GPU
 * 업로드), 로봇 이동 내내 고르게 퍼지면 스트림 도착/그 외 경로. 숫자 없이
 * "버벅인다" 를 콘솔 타임라인으로 데이터화한다. production 빌드 = no-op.
 */
let started = false;

export function startJankMonitor(thresholdMs = 80): void {
  if (
    started ||
    !import.meta.env.DEV ||
    typeof requestAnimationFrame === "undefined"
  ) {
    return;
  }
  started = true;
  let last = performance.now();
  const tick = (t: number) => {
    const dt = t - last;
    last = t;
    if (dt > thresholdMs) {
      console.warn(
        `[jank] frame ${dt.toFixed(0)}ms @ ${new Date().toLocaleTimeString()}`,
      );
    }
    requestAnimationFrame(tick);
  };
  requestAnimationFrame(tick);
}
