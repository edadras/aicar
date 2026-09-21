# Bundled label files (optional)

Empty on purpose — see `../models/README.md`.

Label lists normally live inside a model's descriptor (the `labels` array in
its JSON sidecar, or its entry in `ModelCatalog`). This directory exists for
the case where a model ships a separate `.txt` label file that you would
rather keep alongside the weights; point the descriptor's `labelsPath` at it.

Tracked via `.gitkeep` because `pubspec.yaml` declares it as an asset
directory, and a Flutter build errors on a declared directory that does not
exist.
