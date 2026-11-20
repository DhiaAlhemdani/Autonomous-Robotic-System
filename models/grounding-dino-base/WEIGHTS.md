# Grounding DINO base: weights not in git

`model.safetensors` is **not in this repository** (SR-07: model weights are stored outside git).
A Git LFS pointer had been committed in its place, but the object was never uploaded, so it could not be restored. That pointer has been removed.

| Item | Value |
| --- | --- |
| File | `model.safetensors` |
| Size | 933,400,872 bytes |
| SHA-256 | `5548f844c928c4b6f411fa8cbcc2bfa8dbbba437cb1d513975519f93c2a9ed21` |
| Upstream model | `IDEA-Research/grounding-dino-base` (Hugging Face); paper arXiv:2303.05499 |
| Licence | Apache-2.0 (from the upstream model card; see `README.md` here) |

**To restore:** download the upstream model from the source above, place `model.safetensors` in this folder, and check it with
`sha256sum -c` against the SHA-256 in the table. Do not commit it: `.gitignore` excludes it.

The other files in this folder (config, tokenizer, vocabulary, preprocessor) are small and tracked in git.
