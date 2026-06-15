"""Storage layer — 영속성 인프라 (docs/storage_layer.md).

다른 노드는 SQL/S3 모름. storage_node 가 Zenoh service gateway 로 격리.
Phase 1 = 캘 5종 (RDB only). Phase 2 = scans/meshes/task_runs + ObjectStore 실 사용.
"""
