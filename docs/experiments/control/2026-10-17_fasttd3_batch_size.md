# FastTD3 batch size on the laptop

The FastTD3 paper uses a batch of 32768. With this critic on the laptop's RTX 4080 that costs 100 ms per update. A batch of 8192 costs 18 ms, so the laptop arm runs `FastTd3Config.batch_size` 8192 (`auto_battlebot/rl/learners/fasttd3.py`).
