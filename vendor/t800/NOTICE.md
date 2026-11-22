# Vendored T800 assets

**What this is.** A copy of the T800 configuration and robot-asset files from the upstream EngineAI project, pinned at `third_party/T800-PROJECT` commit `8bfd5b45b571ae6fc8f8c98629c0696e9a4b85c7` (upstream commit `8bfd5b4`). Paths are kept as upstream, so `assets/config/t800/...` and `assets/resource/robot/t800/...` resolve as they do in the submodule. `SHA256SUMS` lists every file.

**Copyright and licence.** Copyright (c) 2025- Shenzhen Zhongqing Robot Technology Co., Ltd. ("EngineAI"). Licensed under the BSD 3-Clause License, reproduced in `LICENSE.txt` in this directory. The upstream copyright and licence are retained.

**Not covered by this notice and not confirmed.**
- **Walking-policy weights** (`assets/config/t800/rl_walking_example/policy/t800_260618_165257_30000.mnn`) and the other `.mnn` policy files. The upstream repo has no separate licence for them. The upstream README says models keep their own terms, and the external document it cites does not state them. Their licence is **unconfirmed**. Do not treat them as BSD-3 without written confirmation from EngineAI.
- Third-party libraries and services, which keep their own terms.
- The `pm01_edu` assets (another robot) and the Grounding DINO model are **not** vendored here.

**Relation to the submodule.** The submodule remains the reference copy. This copy exists because the project owner asked for the T800 resources to be in this repository (`docs/14` §6, O1). If the two diverge, the submodule's pinned commit is the source of truth.

**Verification.** `sha256sum -c SHA256SUMS` from this directory checks the copy against the file list.
