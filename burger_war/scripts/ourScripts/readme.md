# ourScripts

## AR_PoseEstimation.py
相手のARマーカーから相手と自分の位置推定。マーカーが読み取れれば一番正確。①
オドメトリの補助に使う？

## decisionMaking.py
行動を決定

## degreecalc.py
画像に写った赤球から向きを算出？

## enemy_posture.py
画像に写ったマーカーから向きを算出？

## find_marker.py
マーカー画像処理（殿堂入り）

## find_marker_hierarchy.py
マーカー画像処理別ver?（殿堂入り）

## getEnemyPose.py
相手の得点マーカーから相手位置推定。相手が見えなくても相手の位置がわかる。⑤
LiDARの点群から相手位置推定。360度対応。ノイズ多め。④
他の相手の位置推定情報の統合。

## getGreenCirclePosition.py
緑マーカーから相手位置推定。カメラの画角内に入ったらわかる。②

## getRedBallPosition.py
赤玉から相手位置推定。赤玉はスタート位置から見える。すこしズレあり。③

## inputPoint.py
？

## is_field_out.py
フィールド内かどうか

## moveBurger.py
移動

## pointcloudProcessor.py
点群処理（他に統合済み）

## random_pose.py
ロボットをランダムにスポーン？

## sharpness.py
？

## true_pose.py
シミュレータ内のロボットの位置を出力
