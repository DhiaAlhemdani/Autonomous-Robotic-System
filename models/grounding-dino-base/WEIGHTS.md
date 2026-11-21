# Grounding DINO base: files are fetched, not stored in git

The model files are **not** in this repository (SR-07). They are fetched from the upstream Hugging Face repository at a pinned commit and checked against the hashes below.

| Item | Value |
| --- | --- |
| Upstream | `IDEA-Research/grounding-dino-base` (Hugging Face) |
| Pinned revision | `12bdfa3120f3e7ec7b434d90674b3396eccf88eb` (not `main`) |
| Licence | Apache-2.0 (upstream model card) |
| Paper | arXiv:2303.05499 |
| `model.safetensors` | SHA-256 `5548f844c928c4b6f411fa8cbcc2bfa8dbbba437cb1d513975519f93c2a9ed21`, 933,400,872 bytes (Git LFS object in the pinned revision) |
| Small files | Git blob SHA-1 in `scripts/fetch_grounding_dino.py` (`GIT_BLOBS`) |

**To fetch and verify** (repo root):

    pip install huggingface_hub
    python scripts/fetch_grounding_dino.py

**To check files already present:** `python scripts/fetch_grounding_dino.py --verify`

The fetch needs network access to huggingface.co. It is not yet wired into a container image: no perception image exists yet.

**Not verified here:** the sandbox cannot reach huggingface.co, so the download itself has not been run. The hashes were checked against the upstream metadata (the Hugging Face tree API) and against the blob hashes committed earlier.
