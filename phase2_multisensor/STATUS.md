# Status (update at the end of every session)

**Last update:** 2026-10-10
**Current phase:** 2.0 Setup
**Done:** project structure, dataset survey and choice (NCLT primary).

## Next action
1. Disk: over 400 GB free (confirmed 2026-10-10). Budget: ~150 GB for ~8 NCLT sessions without camera.
2. Download ONE short NCLT session for a smoke test: 2013-01-10 (Velodyne ~3 GB, sensors 21 MB,
   ground truth 20 MB). Skip images.
3. Run KISS-ICP on it with `--dataloader nclt`, convert ground truth, evaluate with evo (P2.0a, P2.0b).

## Open questions
- Exact session split (decide in 2.0 after the smoke test).
- KISS-ICP nclt loader layout (checked in its source): data/nclt/<date>/velodyne_sync/<timestamp>.bin and
  data/nclt/ground_truth/groundtruth_<date>.csv; it needs scipy in the kiss-icp env.
- Bako Motors role and the 92% figure for the CV (separate, applications).

## For whoever picks this up (Yassine or Claude)
Read README.md, then this file, then the last entries of DECISIONS.md. Phase 1 lives in the repo root.
