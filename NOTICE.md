# Notices and provenance

New implementation: Copyright (c) 2026 Paul N. Beuchat, MIT.

The OAK pipeline configuration and geometry handling were informed by the
University of Melbourne MIT-licensed `oakd_gpu_cone_detector` at
`5907560cb7a3c5f5df43a9a792243bdee1ce010f`, notably `gpu_session.py` and
`camera_support.py`. Its attribution is retained in `LICENSE.third-party`.
This repository does not distribute its detector or model weights.

DepthAI is a separate dependency, pinned to version 3.9.0,
commit `ac928141ef7750bf3082b5e1c319a115618660e7`.
Its source, examples and dependency licenses are maintained by Luxonis:
https://github.com/luxonis/depthai-core/tree/v3.9.0.

Future cone integration must preserve the model's exact descriptor/hash and
provenance. The legacy model records conflicting AGPL metadata and MIT blob
attribution; the package license alone does not resolve model licensing.
