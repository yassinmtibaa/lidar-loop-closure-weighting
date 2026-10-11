#!/usr/bin/env bash
# usage: ./scripts/download_nclt.sh [date ...]   (default: the 8 split sessions in DECISIONS.md)
# Downloads Velodyne + sensors + ground truth + covariance (no images), unpacks, deletes the archives.
set -e
cd "$(dirname "$0")/../data/nclt"
mkdir -p ground_truth
B=https://s3.us-east-2.amazonaws.com/nclt.perl.engin.umich.edu
DATES=${@:-"2012-01-15 2012-04-29 2012-08-04 2012-05-26 2012-10-28 2012-02-12 2012-06-15 2013-04-05"}
for d in $DATES; do
  echo "=== $d"
  if [ -d "$d/velodyne_sync" ] && [ -f "$d/ms25.csv" ]; then echo "already unpacked, skipping"; continue; fi
  wget -c -q --show-progress $B/velodyne_data/${d}_vel.tar.gz
  wget -c -q --show-progress $B/sensor_data/${d}_sen.tar.gz
  wget -c -q -P ground_truth $B/ground_truth/groundtruth_${d}.csv
  wget -c -q -P ground_truth $B/covariance/cov_${d}.csv
  tar -xzf ${d}_vel.tar.gz && tar -xzf ${d}_sen.tar.gz
  rm -f ${d}_vel.tar.gz ${d}_sen.tar.gz
  echo "$d: $(ls $d/velodyne_sync | wc -l) scans"
done
df -h . | tail -1
