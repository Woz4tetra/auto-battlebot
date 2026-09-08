"""DeepLab field-mask model: build it, load it, and read the metadata beside it.

The field mask is what the C++ DeepLabMaskModel runs at runtime; every real config
selects it. Training and export CLIs over this live in `training/deeplab/`.

load_deeplabv3  model builder, weights loading, and the shared input transforms
model_config    read and write the TOML metadata beside a checkpoint
constants       input geometry and class count the trained model was built with
field_labels    parse a floor-mask filename into field type, scene, and source frame
"""
