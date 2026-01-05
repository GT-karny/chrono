# Chrono::Vehicle FMU インターフェース仕様書

本ドキュメントは、Chrono::Vehicle 連成シミュレーション用 FMU の固定パラメータ (Fixed Parameters)、入力 (Inputs)、出力 (Outputs) をまとめたものです。

## 1. 車両 FMU (`FMU2cs_WheeledVehicle`)
パワートレインやタイヤを含まない、Chrono 車両システム (シャシー、サスペンション、ステアリング、ホイール) をカプセル化します。

### パラメータ (Fixed)
| 変数名 | 型 | 単位 | 説明 |
| :--- | :--- | :--- | :--- |
| `data_path` | String | - | 車両データへのパス |
| `vehicle_JSON` | String | - | 車両 JSON 定義ファイル |
| `system_SMC` | Boolean | - | SMC (Smooth Discrete Element Method) システムの使用 |
| `init_loc` | Vec3 | m | 初期位置 |
| `init_yaw` | Real | rad | 初期ヨー角 |
| `engineblock_dir` | Vec3 | - | エンジンブロック取付方向 |
| `transmissionblock_dir` | Vec3 | - | トランスミッションブロック取付方向 |
| `g_acc` | Vec3 | m/s² | 重力加速度 |
| `step_size` | Real | s | 積分ステップサイズ |
| `out_path` | String | - | 出力ディレクトリ |
| `fps` | Real | - | 描画フレームレート |

### 入力 (Inputs)
| 変数名 | 型 | 単位 | 説明 |
| :--- | :--- | :--- | :--- |
| `steering` | Real | - | ステアリング入力 |
| `throttle` | Real | - | スロットル入力 |
| `braking` | Real | - | ブレーキ入力 |
| `clutch` | Real | - | クラッチ入力 |
| `save_img` | Boolean | - | 画像保存トリガー (Discrete) |
| `driveshaft_torque` | Real | Nm | ドライブシャフトモータートルク |
| `engine_reaction` | Real | Nm | エンジン反力トルク |
| `transmission_reaction` | Real | Nm | トランスミッション反力トルク |
| `wheel_FL.point` | Vec3 | m | 左前輪 荷重作用点 |
| `wheel_FL.force` | Vec3 | N | 左前輪 作用力 |
| `wheel_FL.moment` | Vec3 | Nm | 左前輪 作用モーメント |
| (FR, RL, RR も同様) | | | |

### 出力 (Outputs)
| 変数名 | 型 | 単位 | 説明 |
| :--- | :--- | :--- | :--- |
| `ref_frame` | Frame | m, m/s | 参照フレーム (位置, 回転, 速度, 角速度) |
| `driveshaft_speed` | Real | rad/s | ドライブシャフト角速度 |
| `wheel_FL.pos` | Vec3 | m | 左前輪 位置 |
| `wheel_FL.rot` | Quat | - | 左前輪 回転 (クォータニオン) |
| `wheel_FL.lin_vel` | Vec3 | m/s | 左前輪 並進速度 |
| `wheel_FL.ang_vel` | Vec3 | rad/s | 左前輪 角速度 |
| (FR, RL, RR も同様) | | | |

---

## 2. パワートレイン FMU (`FMU2cs_Powertrain`)
エンジンとトランスミッションシステムをカプセル化します。

### パラメータ (Fixed)
| 変数名 | 型 | 単位 | 説明 |
| :--- | :--- | :--- | :--- |
| `engine_JSON` | String | - | エンジン JSON 定義ファイル |
| `transmission_JSON` | String | - | トランスミッション JSON 定義ファイル |
| `step_size` | Real | s | 積分ステップサイズ |
| `out_path` | String | - | 出力ディレクトリ |

### 入力 (Inputs)
| 変数名 | 型 | 単位 | 説明 |
| :--- | :--- | :--- | :--- |
| `throttle` | Real | - | スロットル入力 |
| `clutch` | Real | - | クラッチ入力 |
| `driveshaft_speed` | Real | rad/s | ドライブシャフト角速度 |

### 出力 (Outputs)
| 変数名 | 型 | 単位 | 説明 |
| :--- | :--- | :--- | :--- |
| `engine_reaction` | Real | Nm | エンジン反力トルク |
| `transmission_reaction` | Real | Nm | トランスミッション反力トルク |
| `driveshaft_torque` | Real | Nm | ドライブシャフトモータートルク |

---

## 3. ドライバー FMU (`FMU2cs_PathFollowerDriver`)
経路追従および速度制御ドライバーをカプセル化します。

### パラメータ (Fixed)
| 変数名 | 型 | 単位 | 説明 |
| :--- | :--- | :--- | :--- |
| `path_file` | String | - | 経路定義ファイル |
| `look_ahead_dist` | Real | - | ステアリング制御 前方注視距離 |
| `Kp_steering` | Real | - | ステアリング制御 比例ゲイン |
| `Ki_steering` | Real | - | ステアリング制御 積分ゲイン |
| `Kd_steering` | Real | - | ステアリング制御 微分ゲイン |
| `throttle_threshold` | Real | - | 速度制御 スロットル/ブレーキ閾値 |
| `Kp_speed` | Real | - | 速度制御 比例ゲイン |
| `Ki_speed` | Real | - | 速度制御 積分ゲイン |
| `Kd_speed` | Real | - | 速度制御 微分ゲイン |
| `step_size` | Real | s | 積分ステップサイズ |
| `out_path` | String | - | 出力ディレクトリ |
| `fps` | Real | - | 描画フレームレート |

### 入力 (Inputs)
| 変数名 | 型 | 単位 | 説明 |
| :--- | :--- | :--- | :--- |
| `ref_frame` | Frame | m, m/s | 参照フレーム (車両から) |
| `target_speed` | Real | m/s | 目標速度 |
| `save_img` | Boolean | - | 画像保存トリガー (Discrete) |

### 出力 (Outputs)
| 変数名 | 型 | 単位 | 説明 |
| :--- | :--- | :--- | :--- |
| `init_loc` | Vec3 | m | 経路始点位置 (Constant) |
| `init_yaw` | Real | rad | 経路始点方位 (Constant) |
| `steering` | Real | - | ステアリング指令値 |
| `throttle` | Real | - | スロットル指令値 |
| `braking` | Real | - | ブレーキ指令値 |

---

## 4. タイヤ FMU (`FMU2cs_ForceElementTire`)
フォースエレメントタイヤモデル (例: TMeasy) をカプセル化します。

### パラメータ (Fixed)
| 変数名 | 型 | 単位 | 説明 |
| :--- | :--- | :--- | :--- |
| `tire_JSON` | String | - | タイヤ JSON 定義ファイル |
| `step_size` | Real | s | 積分ステップサイズ |
| `out_path` | String | - | 出力ディレクトリ |

### 入力 (Inputs)
| 変数名 | 型 | 単位 | 説明 |
| :--- | :--- | :--- | :--- |
| `wheel_state.pos` | Vec3 | m | ホイール位置 |
| `wheel_state.rot` | Quat | - | ホイール回転 |
| `wheel_state.lin_vel` | Vec3 | m/s | ホイール並進速度 |
| `wheel_state.ang_vel` | Vec3 | rad/s | ホイール角速度 |
| `terrain_height` | Real | m | 地形高さ (照会点における) |
| `terrain_normal` | Vec3 | - | 地形法線 (照会点における) |
| `terrain_mu` | Real | - | 地形摩擦係数 |

### 出力 (Outputs)
| 変数名 | 型 | 単位 | 説明 |
| :--- | :--- | :--- | :--- |
| `wheel_load.point` | Vec3 | m | ホイール荷重作用点 |
| `wheel_load.force` | Vec3 | N | ホイール作用力 |
| `wheel_load.moment` | Vec3 | Nm | ホイール作用モーメント |
| `query_point` | Vec3 | m | 地形照会点 (クエリポイント) |

---

## 5. 地形 FMU (`FMU2cs_Terrain`)
地形システム (Flat または RigidMesh) をカプセル化します。

### パラメータ (Fixed)
| 変数名 | 型 | 単位 | 説明 |
| :--- | :--- | :--- | :--- |
| `terrain_type` | String | - | 地形タイプ ("Flat", "RigidMesh") |
| `json_file` | String | - | 地形 JSON ファイル (RigidMesh用) |
| `obj_file` | String | - | 地形 OBJ ファイル (RigidMesh用) |
| `friction` | Real | - | 摩擦係数 (デフォルト/Flat用) |
| `step_size` | Real | s | 積分ステップサイズ |
| `out_path` | String | - | 出力ディレクトリ |

### 入力 (Inputs)
| 変数名 | 型 | 単位 | 説明 |
| :--- | :--- | :--- | :--- |
| `query_point` | Vec3 | m | 照会点 (クエリポイント) 位置 |

### 出力 (Outputs)
| 変数名 | 型 | 単位 | 説明 |
| :--- | :--- | :--- | :--- |
| `height` | Real | m | 地形高さ (照会点における) |
| `normal` | Vec3 | - | 地形法線 (照会点における) |
| `mu` | Real | - | 地形摩擦係数 (照会点における) |
