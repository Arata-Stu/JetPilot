/* Geometry helpers shared by the canvas editor and dependency-free tests. */
const LaneGeometry = (() => {
  const midpoint = (a, b) => [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
  const distance = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);
  function paired(lane) {
    return lane.boundary_mode === "paired" && lane.left_bound.length === lane.right_bound.length;
  }
  function centers(lane) {
    return lane.left_bound.map((p, i) => midpoint(p, lane.right_bound[i]));
  }
  function fromSpine(points, width) {
    if (!Number.isFinite(width) || width <= 0) throw new Error("幅は正の数で指定してください。");
    const left_bound = [], right_bound = [];
    points.forEach((p, i) => {
      const a = points[Math.max(0, i - 1)], b = points[Math.min(points.length - 1, i + 1)];
      const length = distance(a, b);
      const normal = length > 1e-9 ? [-(b[1] - a[1]) / length, (b[0] - a[0]) / length] : [0, 1];
      left_bound.push([p[0] + normal[0] * width / 2, p[1] + normal[1] * width / 2]);
      right_bound.push([p[0] - normal[0] * width / 2, p[1] - normal[1] * width / 2]);
    });
    return { left_bound, right_bound, centerline: points.map(p => [...p]) };
  }
  function split(lane, index, id) {
    if (!paired(lane) || lane.centerline_mode === "manual") throw new Error("分割にはペア境界と自動centerlineが必要です。");
    const n = lane.left_bound.length;
    if (!Number.isInteger(index) || index < 1 || index >= n - 1) throw new Error("端点以外の境界点を選択してください。");
    const a = { ...lane, closed_loop: false }, b = { ...lane, id, primary: false, closed_loop: false };
    for (const field of ["left_bound", "right_bound"]) {
      a[field] = lane[field].slice(0, index + 1).map(p => [...p]);
      b[field] = lane[field].slice(index).map(p => [...p]);
      if (lane.closed_loop) b[field].push([...lane[field][0]]);
    }
    a.centerline = centers(a); b.centerline = centers(b);
    return [a, b];
  }
  function join(a, b) {
    if (a.id === b.id || a.closed_loop || b.closed_loop || !paired(a) || !paired(b)
        || a.centerline_mode === "manual" || b.centerline_mode === "manual") {
      throw new Error("結合には別々の開いたペアレーンと自動centerlineが必要です。");
    }
    if (a.left_bound.length < 2 || b.left_bound.length < 2) throw new Error("各レーンに2組以上の点が必要です。");
    const result = { ...a };
    for (const field of ["left_bound", "right_bound"]) {
      if (distance(a[field].at(-1), b[field][0]) > 0.25) throw new Error("現在レーンの終点と結合先の始点を25cm以内に合わせてください。");
      result[field] = [...a[field].slice(0, -1), midpoint(a[field].at(-1), b[field][0]), ...b[field].slice(1)].map(p => [...p]);
    }
    result.closed_loop = ["left_bound", "right_bound"].every(f => distance(result[f][0], result[f].at(-1)) < 1e-6);
    if (result.closed_loop) for (const f of ["left_bound", "right_bound"]) result[f].pop();
    result.centerline = centers(result);
    return result;
  }
  return { paired, centers, fromSpine, split, join };
})();
if (typeof module !== "undefined") module.exports = LaneGeometry;
