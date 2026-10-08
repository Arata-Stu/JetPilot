"""Render explanatory research figures from saved observations, without refitting.

Requires numpy, matplotlib, Pillow and OpenCV. PNG and SVG share one layout.
Source recording pixels are only cropped/resized; existing overlays are retained.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
from pathlib import Path
from zipfile import ZipFile

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle
import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
BLUE = "#2b5d94"
ORANGE = "#c96a16"
INK = "#152739"
MUTED = "#526578"
LIGHT = "#edf3f9"


def read_json(path):
    return json.loads(path.read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def text(fig, x, y, s, size=14, color=INK, **kw):
    return fig.text(x, y, s, fontsize=size, color=color, va="center", **kw)


def figure_box(fig, x, y, w, h, label, fc=LIGHT, color=BLUE, size=14):
    fig.add_artist(FancyBboxPatch(
        (x, y), w, h, boxstyle="round,pad=0.008,rounding_size=0.008",
        transform=fig.transFigure, facecolor=fc, edgecolor=color, linewidth=1.2))
    text(fig, x+w/2, y+h/2, label, size=size, color=color, ha="center")


def arrow(fig, start, end, color=MUTED):
    fig.add_artist(FancyArrowPatch(start, end, transform=fig.transFigure,
                                  arrowstyle="-|>", mutation_scale=15,
                                  linewidth=1.6, color=color))


def save(fig, stem, out):
    for ext in ("png", "svg"):
        fig.savefig(out/f"{stem}.{ext}", dpi=240, facecolor="white")
    plt.close(fig)


def onset_figure(review_root, out):
    session = "popout-0928-static-100_02"
    folder = review_root/session/"rgb_c001"
    rgb_summary = read_json(folder/"summary.json")
    evs_summary = read_json(review_root/session/"evs_c001"/"summary.json")
    assert rgb_summary["source_decode_strategy"] == "sequential_from_start"
    assert rgb_summary["source_video_sha256"] == evs_summary["source_video_sha256"]
    t0 = rgb_summary["candidate"]["rgb_onset_recording_s"]
    assert t0 == evs_summary["candidate"]["rgb_onset_recording_s"]
    with (folder/"frames.csv").open() as f:
        rows = list(csv.DictReader(f))
    times = np.array([float(r["recording_relative_s"]) for r in rows])
    i0 = int(np.argmin(abs(times-t0)))
    assert abs(times[i0]-t0) < 1e-6
    wanted = [i0-1, i0, i0+1]
    assert wanted[0] >= 0 and wanted[-1] < len(rows)
    assert np.all(np.diff([int(rows[i]["source_frame"]) for i in wanted]) == 1)
    cap = cv2.VideoCapture(str(folder/"candidate_review.mp4"))
    decoded = []
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        decoded.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    cap.release()
    assert len(decoded) == len(rows)
    assert decoded[0].shape == (608, 1280, 3)
    # Identical crop in all three RGB panels (native EVS-view pixel coordinates).
    x0, y0, x1, y1 = 15, 200, 185, 335
    crops = [decoded[i][y0:y1, x0:x1] for i in wanted]
    deltas = (times[wanted]-t0)*1000
    candidate_deltas = {
        "evs": (evs_summary["candidate"]["candidate_recording_s"]-t0)*1000,
        "rgb": (rgb_summary["candidate"]["candidate_recording_s"]-t0)*1000,
    }
    assert 0 < candidate_deltas["evs"] < candidate_deltas["rgb"]
    fig = plt.figure(figsize=(12, 7.3))
    text(fig, .045, .945, "A  RGB初出現を基準に、候補までの時間を測る", 21, BLUE, weight="bold")
    text(fig, .045, .895, "自車静止・左から出現 / static-100_02 / RGBの同一領域を拡大（時間差はこの1記録の例）", 12, MUTED)
    labels = ["直前のRGBフレーム", "RGB初出現（手動注釈）", "次のRGBフレーム"]
    source_entries = []
    for j, (crop, index, dt, label) in enumerate(zip(crops, wanted, deltas, labels)):
        ax = fig.add_axes([.045+j*.318, .515, .274, .302])
        ax.imshow(crop, interpolation="nearest")
        ax.set_xticks([]); ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_color(BLUE if j == 1 else "#bcc8d4")
            spine.set_linewidth(2.7 if j == 1 else 1)
        cx = .045+j*.318+.137
        text(fig, cx, .848, label, 14, BLUE if j == 1 else INK, ha="center", weight="bold")
        tag = "$t_0$ = 0 ms" if j == 1 else f"{dt:+.1f} ms"
        text(fig, cx, .487, tag, 15, BLUE if j == 1 else INK, ha="center")
        if j == 1:
            ax.annotate("車体の先端", xy=(62, 72), xytext=(116, 34),
                        fontsize=12, color="white", ha="center",
                        bbox=dict(boxstyle="round,pad=.25", fc=BLUE, ec="none"),
                        arrowprops=dict(arrowstyle="->", color="white", lw=1.6))
        crop_path = out/"source_crops"/f"{session}_rgb_frame_{rows[index]['source_frame']}.png"
        crop_path.parent.mkdir(exist_ok=True)
        Image.fromarray(crop).save(crop_path)
        source_entries.append(dict(output=crop_path.name, source_frame=int(rows[index]["source_frame"]),
            clip_frame=index, recording_time_s=float(times[index]), onset_delta_ms=float(dt),
            crop_xyxy=[x0,y0,x1,y1], sha256=sha(crop_path)))
    ax = fig.add_axes([.085, .158, .85, .235])
    ax.set(xlim=(-20, 50), ylim=(-.72, 1.3))
    ax.axis("off")
    ax.annotate("", xy=(49, .4), xytext=(-19, .4), arrowprops=dict(arrowstyle="->", lw=1.4, color=MUTED))
    for t in (-16.757, 0, 16.833, 33.589):
        ax.plot([t,t],[.34,.46],color="#b1becb",lw=1)
    ax.axvline(0, ymin=.07, ymax=.93, color=MUTED, lw=1.2, ls="--")
    ax.text(0, 1.16, "$t_0$：RGB初出現", ha="center", fontsize=13, color=INK)
    for method,y,color in (("evs",-.10,ORANGE),("rgb",-.50,BLUE)):
        dt = candidate_deltas[method]
        ax.plot([dt,dt],[y,.4],color=color,lw=1.6)
        ax.scatter([dt],[.4],color=color,s=48,zorder=5)
        ax.annotate("",xy=(dt,y),xytext=(0,y),arrowprops=dict(arrowstyle="<->",lw=1.5,color=color))
        ax.text(dt+.9,y,rf"$\Delta t_{{\mathrm{{{method.upper()}}}}}$ = {dt:.1f} ms",fontsize=13,color=color,va="center")
        ax.text(dt,.68,f"$t_{{\\mathrm{{{method.upper()}}}}}$",ha="center",fontsize=14,color=color)
    ax.text(49,.64,"時間",ha="right",fontsize=12,color=MUTED)
    text(fig,.045,.095,"t₀は注釈したRGBフレームの時刻。RGBは約60 Hz（約16.7 ms間隔）で、物理的な出現時刻とは区別する。",11.5,MUTED)
    text(fig,.045,.055,"水色枠は元の確認動画にある候補タイル。追跡枠ではない。候補時刻は解析値であり、表示フレーム時刻とは異なる。",10.5,MUTED)
    save(fig,"01_rgb_onset_and_timing",out)
    return dict(session=session, purpose="Annotated RGB onset and actual per-record candidate times; illustrative static recording, not an average.",
        annotation_t0_s=t0, candidate_minus_onset_ms=candidate_deltas,
        crop_entries=source_entries, source_files={str(p):sha(p) for p in [folder/"summary.json",folder/"frames.csv",folder/"candidate_review.mp4",review_root/session/"evs_c001"/"summary.json"]},
        limitations=["Existing time annotation retained; no onset relabeling.",
                    "Video decoded sequentially; identical crops and existing tile overlays retained.",
                    "These are offline candidate timestamps, not end-to-end response latency."])


def background_figure(bundle, maps_root, out):
    session="test_11"
    npz_path=maps_root/session/"evs_background_maps.npz"
    with np.load(npz_path,allow_pickle=False) as z:
        r={k:z[k] for k in z.files}
    with ZipFile(bundle) as archive:
        meta= json.loads(archive.read(f"tile_dev_trial01/{session}/tiles.json"))
        with np.load(io.BytesIO(archive.read(f"tile_dev_trial01/{session}/evs_tiles.npz")),allow_pickle=False) as z:
            data={k:z[k] for k in z.files}
        bundle_tile_sha=hashlib.sha256(archive.read(f"tile_dev_trial01/{session}/evs_tiles.npz")).hexdigest()
    assert np.array_equal(r["tile_id"],data["tile_id"])
    assert np.allclose(r["time_s"],data["time_s"],rtol=0,atol=1e-9)
    with (maps_root/"candidates.csv").open() as f:
        c=next(row for row in csv.DictReader(f) if row["session"]==session and row["method"]=="evs" and row["candidate"]=="1")
    i=int(np.flatnonzero(r["alarm"])[0])
    assert abs(float(r["time_s"][i])-float(c["start_relative_time_s"]))<1e-8
    pair_ids=[int(c["pair_tile_a"]),int(c["pair_tile_b"])]
    assert list(r["tile_id"][r["winner_pair"][i]]) == pair_ids
    width,height=meta["output_size"]
    maps=[]
    density=data["counts"][i]/data["valid_pixels"]
    for values in (density,r["predicted_density"][i],r["residual_z"][i]):
        arr=np.full((height//32,width//32),np.nan)
        for tile,v in zip(meta["tiles"],values):
            arr[tile["y"]//32,tile["x"]//32]=v
        maps.append(arr)
    fig=plt.figure(figsize=(12,7.3))
    text(fig,.045,.945,"B  背景活動から、局所的に残る反応を取り出す",21,BLUE,weight="bold")
    text(fig,.045,.895,"実データ例：自車直進・調整用 test_11（EVS） /  32×32画素のグリッド",12.5,MUTED)
    text(fig,.045,.846,"背景モデルは飛び出しなし記録から作成・固定し、現在の活動マップに当てはめる。",13,INK)
    labels=["① 観測した活動", "② 推定した背景活動", "③ 背景に対する局所残差"]
    vmax=max(float(density.max()),float(r["predicted_density"][i].max()))
    cmap=plt.get_cmap("magma").copy();cmap.set_bad("#edf0f3")
    axes=[];ims=[]
    for j,(arr,title) in enumerate(zip(maps,labels)):
        ax=fig.add_axes([.056+j*.318,.445,.266,.274])
        im=ax.imshow(arr,extent=(0,width,height,0),interpolation="nearest",aspect="equal",
                     cmap=cmap,vmin=0,vmax=vmax if j<2 else 10)
        ax.set(ylim=(342,70),xticks=[0,320,640],yticks=[96,224,342])
        ax.tick_params(labelsize=10,length=3,colors=MUTED)
        for spine in ax.spines.values(): spine.set_color("#a5b2bf")
        text(fig,.056+j*.318+.133,.766,title,14,BLUE,ha="center",weight="bold")
        ax.set_xlabel("EVS座標 x [px]",fontsize=10,color=MUTED,labelpad=2)
        if j==0: ax.set_ylabel("y [px]",fontsize=10,color=MUTED)
        axes.append(ax);ims.append(im)
    for tid in pair_ids:
        tile=next(t for t in meta["tiles"] if int(t["tile_id"])==tid)
        axes[2].add_patch(Rectangle((tile["x"],tile["y"]),32,32,fill=False,ec="#67e1ed",lw=1.6))
    axes[2].annotate("候補タイル",xy=(32,282),xytext=(180,190),color="white",fontsize=11,
                     arrowprops=dict(arrowstyle="->",color="white",lw=1.3))
    for j in (0,1):
        cax=fig.add_axes([.085+j*.318,.350,.207,.014])
        cb=fig.colorbar(ims[j],cax=cax,orientation="horizontal")
        cb.ax.tick_params(labelsize=9,length=2)
        cb.set_label("イベント数 / 有効画素数（2 ms）",fontsize=10,labelpad=2)
    cax=fig.add_axes([.721,.350,.207,.014])
    cb=fig.colorbar(ims[2],cax=cax,orientation="horizontal",ticks=[0,5,10])
    cb.ax.tick_params(labelsize=9,length=2);cb.set_label("標準化した正の残差 z",fontsize=10,labelpad=2)
    figure_box(fig,.055,.168,.252,.075,"正の残差を時間積算")
    figure_box(fig,.373,.168,.252,.075,"隣接2タイルで判定")
    figure_box(fig,.691,.168,.252,.075,"検出候補",fc="#fff3e6",color=ORANGE)
    arrow(fig,(.319,.206),(.358,.206));arrow(fig,(.637,.206),(.676,.206))
    dt=float(c["minus_onset_ms"])
    text(fig,.045,.103,f"図示時刻：この記録のEVS候補時刻（RGB初出現の {dt:.1f} ms 後）。方法説明のための調整用例。",11.5,MUTED)
    text(fig,.045,.062,"左・中央は密度、右は特徴差を標準化した残差で、尺度が異なる。背景に対応する反応も一部残る。",11,MUTED)
    save(fig,"02_grid_background_activity",out)
    np.savez_compressed(out/"source_maps.npz",observed_density=maps[0],estimated_density=maps[1],positive_standardized_residual=maps[2])
    return dict(session=session,subset="development",method="evs",purpose="Method illustration, not evaluation success evidence.",
        index=i,candidate_recording_s=float(r["time_s"][i]),candidate_minus_rgb_onset_ms=dt,pair_tile_ids=pair_ids,
        view_limits_xy=[0,640,70,342],density_color_limits=[0,vmax],residual_color_limits=[0,10],
        bundle_sha256=sha(bundle),bundle_tile_sha256=bundle_tile_sha,npz_sha256=sha(npz_path),
        candidates_sha256=sha(maps_root/"candidates.csv"),
        computation="Observed density=n/A. Predicted density and positive standardized sqrt-density residual read from saved detector arrays; no refitting or inference rerun.")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--review-root",type=Path,default=Path("/Users/at/Downloads/scp/grid_background_static_review_trial02"))
    parser.add_argument("--bundle",type=Path,default=REPO/"record/09-30/analysis/development_debug_bundle01.zip")
    parser.add_argument("--maps-root",type=Path,default=REPO/"record/09-30/analysis/grid_background_dev_20261006")
    parser.add_argument("--output",type=Path,default=HERE)
    parser.add_argument("--font",type=Path,default=Path("/System/Library/Fonts/ヒラギノ角ゴシック W3.ttc"))
    args=parser.parse_args()
    if not args.font.is_file():
        args.font=Path("/System/Library/Fonts/Supplemental/Arial Unicode.ttf")
    font_manager.fontManager.addfont(str(args.font))
    family=font_manager.FontProperties(fname=str(args.font)).get_name()
    plt.rcParams.update({"font.family":family,"axes.unicode_minus":False,"svg.fonttype":"path","savefig.facecolor":"white"})
    args.output.mkdir(parents=True,exist_ok=True)
    provenance={"figure_a":onset_figure(args.review_root,args.output),
                "figure_b":background_figure(args.bundle,args.maps_root,args.output)}
    provenance["code_sha256"]=sha(Path(__file__))
    provenance["outputs"]={p.name:sha(p) for p in sorted(args.output.iterdir()) if p.suffix in (".svg",".png",".npz")}
    (args.output/"provenance.json").write_text(json.dumps(provenance,ensure_ascii=False,indent=2)+"\n")
    print(f"Saved two figures as PNG/SVG: {args.output}")


if __name__ == "__main__":
    main()
