# Synthetic Scenario Spec

## Scenario A

- 主 target は近傍を滑らかに移動
- occlusion を短 / 中 / 長で混ぜる
- bbox size は微小揺れを持つ
- competing candidate は近傍に出し、size を少しずらす

## Scenario B Legacy

- 既存 stress 条件の再現
- clutter bbox は固定寸法
- same-class clutter を高頻度に混ぜる

## Scenario B Variable BBox

- clutter bbox を size / aspect ratio 可変に拡張
- size: 小 / 中 / 大
- aspect ratio: 正方 / 縦長 / 横長
- 一部 clutter を true target 近傍へ出す
- same-class / other-class を混在させる
- clutter 寿命は 1-3 frame 中心

## Fixed Seeds

- Scenario A: `1234`
- Scenario B Legacy: `5678`
- Scenario B Production-like: `5678`
- Scenario B Variable BBox: `9012`
