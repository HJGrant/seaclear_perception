# Deleted Files

以下は今回削除した local compatibility shim であり、最終版全文は存在しない。

- `src/marine_acoustic_msgs/CMakeLists.txt`
- `src/marine_acoustic_msgs/package.xml`
- `src/marine_acoustic_msgs/msg/Dvl.msg`
- `src/marine_acoustic_msgs/msg/DvlBeam.msg`

削除理由:

- recorded bag の `marine_acoustic_msgs/msg/Dvl` 実 schema と不一致で runtime kill を起こしたため

