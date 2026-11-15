# Config extends and loading

## Effective config dump

`load_merged_table(path)` loads `path` and resolves its `extends` chain into one merged TOML table, which is exactly the data fed to every section parser. It is useful for dumping the effective config. It reflects only values explicitly set by the config or its bases: defaults the parsers apply for omitted optional fields are not included.

## Label to keypoint mapping format

The label-to-keypoints config is an array of tables, which guarantees order:

```toml
[[section]]
label = "LABEL_NAME"
keypoints = ["KEYPOINT_LABEL_1", "KEYPOINT_LABEL_2"]
```

Storage is ordered so indices match the model's class order.
