# jetpilot_signal_detection

YOLO one-stage の `vision_msgs/Detection2DArray` を、HD map 上で有効な区間だけ時系列投票し、安定した左右・直進判断へ変換する。

学習クラス名は `arrow_left`, `arrow_straight`, `arrow_right`。学習時の class index 順序と decoder の `class_names` を必ず一致させる。
