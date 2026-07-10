/**
 * WaypointPanel — Robot Asset Layer UI (RobotAssetsMode 코어).
 *
 * 두 탭:
 *   - Library : 현재 joint 자세(rad) 티칭 저장 + 목록(이름변경/삭제)
 *   - Groups  : 목적별 묶음(Search/Scan/...) 생성 + 멤버 추가/제거/순서변경(up/down)
 *
 * 티칭 소스 = backend WaypointModule 이 Motion.TcpState(rad) 를 캐시 → TEACH 는
 * "현재 joint 저장". 이 패널은 현재 자세를 tcp stream 으로 보여주기만 (참고용).
 * docs/backend_v2.md §17.2.
 */
import { useCallback, useEffect, useState } from "react";
import { Button } from "@/components/ui/button";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { useRobotId } from "@/hooks/useRobotId";
import { useBridgeConnected, useService, useStream } from "@/framework";
import { ServiceKey, Topic } from "@/api/generated/contract";
import type {
  WaypointGroupRecord,
  WaypointRecord,
} from "@/api/generated/contract";
import { useWaypointStore } from "@/stores/waypointStore";

export function WaypointPanel() {
  const robotId = useRobotId();

  const connected = useBridgeConnected();

  const teachSvc = useService(ServiceKey.WAYPOINT_TEACH, robotId);
  const moveJSvc = useService(ServiceKey.MOTION_MOVE_J, robotId);
  const listSvc = useService(ServiceKey.WAYPOINT_LIST, robotId);
  const renameSvc = useService(ServiceKey.WAYPOINT_RENAME, robotId);
  const deleteSvc = useService(ServiceKey.WAYPOINT_DELETE, robotId);
  const createGroupSvc = useService(ServiceKey.WAYPOINT_CREATE_GROUP, robotId);
  const listGroupsSvc = useService(ServiceKey.WAYPOINT_LIST_GROUPS, robotId);
  const deleteGroupSvc = useService(ServiceKey.WAYPOINT_DELETE_GROUP, robotId);
  const addMemberSvc = useService(ServiceKey.WAYPOINT_ADD_TO_GROUP, robotId);
  const removeMemberSvc = useService(ServiceKey.WAYPOINT_REMOVE_FROM_GROUP, robotId);
  const reorderSvc = useService(ServiceKey.WAYPOINT_REORDER_GROUP, robotId);
  const listMembersSvc = useService(ServiceKey.WAYPOINT_LIST_GROUP_MEMBERS, robotId);

  const tcp = useStream(Topic.MOTION_TCP_STATE, { robotId });

  const [waypoints, setWaypoints] = useState<WaypointRecord[]>([]);
  const [groups, setGroups] = useState<WaypointGroupRecord[]>([]);
  const [selectedGroup, setSelectedGroup] = useState<number | null>(null);
  const [members, setMembers] = useState<WaypointRecord[]>([]);
  const [name, setName] = useState("");
  const [groupName, setGroupName] = useState("");
  const [editingId, setEditingId] = useState<number | null>(null);
  const [editName, setEditName] = useState("");
  const [msg, setMsg] = useState("");

  // waypoint 는 robot-agnostic — 생성/목록은 req 에 robot_id, 나머지는 row id 파생.
  const refreshWaypoints = useCallback(async () => {
    const res = await listSvc.call({ robot_id: robotId });
    setWaypoints(
      (res.data as { waypoints?: WaypointRecord[] } | null)?.waypoints ?? [],
    );
  }, [listSvc, robotId]);

  const refreshGroups = useCallback(async () => {
    const res = await listGroupsSvc.call({ robot_id: robotId });
    setGroups(
      (res.data as { groups?: WaypointGroupRecord[] } | null)?.groups ?? [],
    );
  }, [listGroupsSvc, robotId]);

  const refreshMembers = useCallback(
    async (gid: number) => {
      const res = await listMembersSvc.call({ group_row_id: gid });
      setMembers(
        (res.data as { waypoints?: WaypointRecord[] } | null)?.waypoints ?? [],
      );
    },
    [listMembersSvc],
  );

  useEffect(() => {
    // 초기 목록 로드 — WS 미연결 시 callService 가 drop → timeout 후 빈 목록.
    // connected 를 dep 으로 두어 연결 완료(및 리로드 시 재연결) 후 fetch.
    // (mirror.ts / capability.ts 와 동일 패턴.)
    if (!connected) return;
    // setState 는 await 이후(비동기)라 동기 cascading render 아님.
    /* eslint-disable react-hooks/set-state-in-effect */
    void refreshWaypoints();
    void refreshGroups();
    /* eslint-enable react-hooks/set-state-in-effect */
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [robotId, connected]);

  // ── waypoint CRUD ──────────────────────────────────────────
  const onTeach = async () => {
    const nm = name.trim();
    if (!nm) return;
    const res = await teachSvc.call({ robot_id: robotId, name: nm });
    const d = res.data as { accepted?: boolean } | null;
    if (d?.accepted) {
      setName("");
      setMsg(`저장: ${nm}`);
      await refreshWaypoints();
    } else {
      setMsg(`실패: ${res.message}`);
    }
  };

  const onDelete = async (wid: number) => {
    await deleteSvc.call({ waypoint_row_id: wid });
    await refreshWaypoints();
    if (selectedGroup != null) await refreshMembers(selectedGroup);
  };

  // MoveJ 로 저장된 joint 자세 재현 (Ruckig trajectory).
  const onGoto = async (wp: WaypointRecord) => {
    const res = await moveJSvc.call({ target_joints: wp.joint_values });
    const d = res.data as { accepted?: boolean } | null;
    setMsg(d?.accepted ? `이동: ${wp.name}` : `이동 실패: ${res.message}`);
  };

  // ── ghost 미리보기 (3D, WaypointScenePart) ─────────────────
  // 명시적 [보기] 토글 — hover X. 렌더는 scenePart, 여기는 store 만.
  const ghostPreview = useWaypointStore((s) => s.previews[robotId]);
  const setPreview = useWaypointStore((s) => s.setPreview);
  const onTogglePreview = (wp: WaypointRecord) => {
    if (wp.id == null) return;
    if (ghostPreview?.waypointId === wp.id) {
      setPreview(robotId, null); // 같은 항목 재클릭 = 끄기
    } else {
      setPreview(robotId, {
        waypointId: wp.id,
        name: wp.name,
        jointNames: wp.joint_names,
        jointAngles: wp.joint_values,
      });
    }
  };
  // 패널 unmount / robot 스위칭 시 ghost 정리 (남은 ghost = 주인 없는 표시)
  useEffect(() => {
    return () => setPreview(robotId, null);
  }, [robotId, setPreview]);

  const startEdit = (wp: WaypointRecord) => {
    if (wp.id == null) return;
    setEditingId(wp.id);
    setEditName(wp.name);
  };

  const cancelEdit = () => {
    setEditingId(null);
    setEditName("");
  };

  const onRename = async (wid: number) => {
    const nm = editName.trim();
    if (!nm) return;
    const res = await renameSvc.call({ waypoint_row_id: wid, name: nm });
    const d = res.data as { ok?: boolean } | null;
    if (d?.ok) {
      cancelEdit();
      await refreshWaypoints();
      if (selectedGroup != null) await refreshMembers(selectedGroup);
    } else {
      setMsg(`이름 변경 실패: ${res.message}`);
    }
  };

  // ── group ──────────────────────────────────────────────────
  const onCreateGroup = async () => {
    const nm = groupName.trim();
    if (!nm) return;
    const res = await createGroupSvc.call({ robot_id: robotId, name: nm });
    const d = res.data as { accepted?: boolean } | null;
    if (d?.accepted) {
      setGroupName("");
      await refreshGroups();
    } else {
      setMsg(`group 실패: ${res.message}`);
    }
  };

  const onSelectGroup = async (gid: number) => {
    setSelectedGroup(gid);
    await refreshMembers(gid);
  };

  const onDeleteGroup = async (gid: number) => {
    await deleteGroupSvc.call({ group_row_id: gid });
    if (selectedGroup === gid) {
      setSelectedGroup(null);
      setMembers([]);
    }
    await refreshGroups();
  };

  const onAddMember = async (wid: number) => {
    if (selectedGroup == null) return;
    await addMemberSvc.call({ group_row_id: selectedGroup, waypoint_row_id: wid });
    await refreshMembers(selectedGroup);
  };

  const onRemoveMember = async (wid: number) => {
    if (selectedGroup == null) return;
    await removeMemberSvc.call({
      group_row_id: selectedGroup,
      waypoint_row_id: wid,
    });
    await refreshMembers(selectedGroup);
  };

  const onMove = async (idx: number, dir: -1 | 1) => {
    if (selectedGroup == null) return;
    const j = idx + dir;
    if (j < 0 || j >= members.length) return;
    const next = [...members];
    [next[idx], next[j]] = [next[j], next[idx]];
    setMembers(next); // optimistic
    const ordered = next
      .map((m) => m.id)
      .filter((x): x is number => x != null);
    await reorderSvc.call({
      group_row_id: selectedGroup,
      ordered_waypoint_row_ids: ordered,
    });
    await refreshMembers(selectedGroup);
  };

  const memberIds = new Set(members.map((m) => m.id));
  const addable = waypoints.filter((w) => !memberIds.has(w.id));
  const joints = tcp.value?.joints ?? null;

  return (
    <div
      className="h-full overflow-y-auto p-3 text-[12px]"
      data-testid="waypoint-panel"
    >
      <Tabs defaultValue="library" className="gap-3">
        <TabsList className="w-full">
          <TabsTrigger value="library" data-testid="tab-library">
            Waypoint Library
          </TabsTrigger>
          <TabsTrigger value="groups" data-testid="tab-groups">
            Waypoint Groups
          </TabsTrigger>
        </TabsList>

        {/* ── Library ── */}
        <TabsContent value="library">
          <section className="mb-3">
            <div className="mb-1 font-mono uppercase text-muted-foreground">
              teach current pose
            </div>
            <p className="mb-1 truncate font-mono text-[10px] text-muted-foreground">
              {joints
                ? joints.map((j) => j.toFixed(2)).join(", ")
                : "joint state 대기…"}
            </p>
            <div className="flex gap-2">
              <input
                value={name}
                onChange={(e) => setName(e.target.value)}
                placeholder="이름 (예: search_left)"
                data-testid="wp-name"
                className="min-w-0 flex-1 rounded border border-zinc-700 bg-zinc-900 px-2 py-1 font-mono"
              />
              <Button size="sm" onClick={onTeach} data-testid="wp-teach">
                티칭 저장
              </Button>
            </div>
          </section>

          <section>
            <div className="mb-1 font-mono uppercase text-muted-foreground">
              waypoints ({waypoints.length})
            </div>
            <div className="flex flex-col gap-1" data-testid="wp-list">
              {waypoints.length === 0 ? (
                <span className="text-muted-foreground">없음</span>
              ) : (
                waypoints.map((w) => (
                  <div
                    key={w.id}
                    className="flex items-center gap-2 rounded border border-zinc-700 px-2 py-1"
                    data-testid="wp-row"
                  >
                    {editingId === w.id ? (
                      <>
                        <input
                          value={editName}
                          onChange={(e) => setEditName(e.target.value)}
                          onKeyDown={(e) => {
                            if (e.key === "Enter" && w.id != null) onRename(w.id);
                            else if (e.key === "Escape") cancelEdit();
                          }}
                          autoFocus
                          data-testid="wp-edit-name"
                          className="min-w-0 flex-1 rounded border border-zinc-700 bg-zinc-900 px-1 font-mono"
                        />
                        <Button
                          size="sm"
                          variant="outline"
                          onClick={() => w.id != null && onRename(w.id)}
                          data-testid="wp-rename-save"
                        >
                          저장
                        </Button>
                        <Button
                          size="sm"
                          variant="ghost"
                          onClick={cancelEdit}
                          data-testid="wp-rename-cancel"
                        >
                          취소
                        </Button>
                      </>
                    ) : (
                      <>
                        <span className="flex-1 truncate font-mono">{w.name}</span>
                        <Button
                          size="sm"
                          variant={
                            ghostPreview?.waypointId === w.id ? "default" : "ghost"
                          }
                          onClick={() => onTogglePreview(w)}
                          data-testid="wp-preview"
                        >
                          보기
                        </Button>
                        <Button
                          size="sm"
                          variant="outline"
                          onClick={() => onGoto(w)}
                          data-testid="wp-goto"
                        >
                          이동
                        </Button>
                        <Button
                          size="sm"
                          variant="ghost"
                          onClick={() => startEdit(w)}
                          data-testid="wp-rename"
                        >
                          이름
                        </Button>
                        <Button
                          size="sm"
                          variant="ghost"
                          onClick={() => w.id != null && onDelete(w.id)}
                          data-testid="wp-delete"
                        >
                          삭제
                        </Button>
                      </>
                    )}
                  </div>
                ))
              )}
            </div>
          </section>
        </TabsContent>

        {/* ── Groups ── */}
        <TabsContent value="groups">
          <section className="mb-3">
            <div className="mb-1 font-mono uppercase text-muted-foreground">
              groups
            </div>
            <div className="mb-2 flex gap-2">
              <input
                value={groupName}
                onChange={(e) => setGroupName(e.target.value)}
                placeholder="group 이름 (예: search)"
                data-testid="wp-group-name"
                className="min-w-0 flex-1 rounded border border-zinc-700 bg-zinc-900 px-2 py-1 font-mono"
              />
              <Button
                size="sm"
                onClick={onCreateGroup}
                data-testid="wp-create-group"
              >
                group 생성
              </Button>
            </div>
            <div className="flex flex-col gap-1" data-testid="wp-group-list">
              {groups.length === 0 ? (
                <span className="text-muted-foreground">없음</span>
              ) : (
                groups.map((g) => (
                  <div
                    key={g.id}
                    className={`flex items-center gap-2 rounded border px-2 py-1 ${
                      g.id === selectedGroup
                        ? "border-emerald-500"
                        : "border-zinc-700"
                    }`}
                  >
                    <button
                      onClick={() => g.id != null && onSelectGroup(g.id)}
                      className="flex-1 truncate text-left font-mono"
                      data-testid="wp-group-select"
                    >
                      {g.name}
                    </button>
                    <Button
                      size="sm"
                      variant="ghost"
                      onClick={() => g.id != null && onDeleteGroup(g.id)}
                      data-testid="wp-group-delete"
                    >
                      삭제
                    </Button>
                  </div>
                ))
              )}
            </div>
          </section>

          {selectedGroup != null && (
            <>
              <section className="mb-3">
                <div className="mb-1 font-mono uppercase text-muted-foreground">
                  members (순서)
                </div>
                <div className="flex flex-col gap-1" data-testid="wp-member-list">
                  {members.length === 0 ? (
                    <span className="text-muted-foreground">비어있음</span>
                  ) : (
                    members.map((m, idx) => (
                      <div
                        key={m.id}
                        className="flex items-center gap-1 rounded border border-zinc-700 px-2 py-1"
                        data-testid="wp-member-row"
                      >
                        <span className="w-4 text-right font-mono text-muted-foreground">
                          {idx + 1}
                        </span>
                        <span className="flex-1 truncate font-mono">{m.name}</span>
                        <Button
                          size="sm"
                          variant="ghost"
                          disabled={idx === 0}
                          onClick={() => onMove(idx, -1)}
                          data-testid="wp-member-up"
                        >
                          ↑
                        </Button>
                        <Button
                          size="sm"
                          variant="ghost"
                          disabled={idx === members.length - 1}
                          onClick={() => onMove(idx, 1)}
                          data-testid="wp-member-down"
                        >
                          ↓
                        </Button>
                        <Button
                          size="sm"
                          variant="ghost"
                          onClick={() => m.id != null && onRemoveMember(m.id)}
                          data-testid="wp-member-remove"
                        >
                          ✕
                        </Button>
                      </div>
                    ))
                  )}
                </div>
              </section>

              <section>
                <div className="mb-1 font-mono uppercase text-muted-foreground">
                  add to group
                </div>
                <div className="flex flex-col gap-1" data-testid="wp-addable-list">
                  {addable.length === 0 ? (
                    <span className="text-muted-foreground">추가할 waypoint 없음</span>
                  ) : (
                    addable.map((w) => (
                      <button
                        key={w.id}
                        onClick={() => w.id != null && onAddMember(w.id)}
                        className="rounded border border-zinc-700 px-2 py-1 text-left font-mono hover:border-emerald-500"
                        data-testid="wp-add-member"
                      >
                        + {w.name}
                      </button>
                    ))
                  )}
                </div>
              </section>
            </>
          )}
        </TabsContent>
      </Tabs>

      <div className="mt-3 text-muted-foreground" data-testid="wp-msg">
        {msg}
      </div>
    </div>
  );
}
