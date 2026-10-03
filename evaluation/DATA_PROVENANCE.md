# Recorded source selection and license

`train-selection-listing.xml` and `val-selection-listing.xml` retain the public
S3 listing responses used to select this subset. `sources.json` contains the
first six lexicographically listed parquet objects from each split, with exact
URLs, byte sizes and SHA-256. The evaluator verifies these bytes and never changes
the selected objects based on results. This is a fixed small inventory, not an
externally timestamped preregistration or representative sample.

The original public listings are accessible with S3 ListObjectsV2 under:

- `https://argoverse.s3.amazonaws.com/?list-type=2&prefix=datasets%2Fav2%2Fmotion-forecasting%2Ftrain%2F&max-keys=24`
- `https://argoverse.s3.amazonaws.com/?list-type=2&prefix=datasets%2Fav2%2Fmotion-forecasting%2Fval%2F&max-keys=24`

The official [Argoverse 2 dataset page](https://www.argoverse.org/av2.html)
explicitly specifies CC BY-NC-SA 4.0 for the data and MIT for code/APIs. Its
official citation is NeurIPS Datasets and Benchmarks 2021. The paper's
[arXiv upload](https://arxiv.org/abs/2301.00493) was in January 2023; that is a
different date. These statements were checked on 2026-10-03. Full website/terms
text is not redistributed. [DATA_LICENSE.md](../DATA_LICENSE.md) preserves the
data attribution and terms for the dataset-derived trials and replays.
