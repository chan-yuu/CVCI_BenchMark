#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import argparse
from huggingface_hub import snapshot_download


def main():
    parser = argparse.ArgumentParser(description="Download CVCI_BENCHmark dataset from Hugging Face.")
    parser.add_argument(
        "--local-dir",
        type=str,
        default="./CVCI_BENCHmark",
        help="Local directory to save the dataset (default: ./CVCI_BENCHmark)",
    )
    args = parser.parse_args()

    repo_id = "55sleeper/CVCI_BENCHmark"
    local_dir = os.path.abspath(args.local_dir)

    print("Starting download of CVCI_BENCHmark dataset...")
    print("Repository: {}".format(repo_id))
    print("Target directory: {}".format(local_dir))

    try:
        snapshot_download(
            repo_id=repo_id,
            repo_type="dataset",
            local_dir=local_dir,
            resume_download=True,
        )
        print("Dataset downloaded successfully to: {}".format(local_dir))
    except Exception as e:
        print("Download failed: {}".format(e))
        raise


if __name__ == "__main__":
    main()
