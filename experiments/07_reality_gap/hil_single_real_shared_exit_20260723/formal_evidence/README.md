# Stage 4 formal evidence

`stage4_20260803_144220/` is the canonical evidence directory for the bounded
physical HIL result reported in the dissertation. It was copied from
`/home/eamon/epuck_comm_bags/hil_stage4_20260803_144220/` without changing the
evidence files.

The topology was one physical e-puck2 and one software-only virtual peer. The
directory must not be interpreted as evidence from two physical robots or as a
statistical simulation-to-reality comparison.

Run the repository-level read-only verifier from the repository root:

```bash
python3 tools/verify_evidence.py --stage4-only
```

`FINAL_SHA256SUMS.txt` is authoritative for the final evidence set. The files
contain environment-specific technical metadata released with the author's
explicit approval, but no passwords, private keys or API credentials.
