# jetpilot_recovery_planner

通常走行中の odometry を breadcrumb として保持し、衝突復帰要求時に来た軌跡を逆順へ並べた後退軌道を生成する。単純な直線後退ではないため、controller は過去のコース形状に沿ってステアしながら戻り、復帰終了時の車体向きを元のコース方向へ合わせやすい。

後方障害物センサはこの planner の範囲外であり、実車では `/safety/collision_detected` の解除とは別に reverse inhibit を車両 safety 層へ追加すること。
