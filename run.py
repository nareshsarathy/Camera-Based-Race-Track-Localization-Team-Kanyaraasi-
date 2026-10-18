#!/usr/bin/env python3
"""
Camera-based Localization on a Race Track (Yas Marina Circuit)
Constructor Labs — National AI Hackathon 2026 by SchubKI

Entrypoint script:
    python run.py --mcap <path_to_mcap> --out <path_to_output_csv>
"""
import argparse
import sys
from pathlib import Path

# Add src to python path
sys.path.insert(0, str(Path(__file__).resolve().parent))

from src.pipeline import run_localization
from src import config

def main():
    parser = argparse.ArgumentParser(
        description="Run camera-based localization on Yas Marina race track MCAP bag."
    )
    parser.add_argument(
        "--mcap",
        type=str,
        default=str(config.BASE_DIR / "c0_2026-10-13_22-53-42_camloc" / "c0_2026-10-13_22-53-42_camloc_0.mcap"),
        help="Path to input MCAP bag file."
    )
    parser.add_argument(
        "--out",
        type=str,
        default=str(config.OUTPUTS_DIR / "submission.csv"),
        help="Path to write the output pose CSV file."
    )
    parser.add_argument(
        "--bounds",
        type=str,
        default=str(config.TRACK_BOUNDS_PATH),
        help="Path to track_bounds.json."
    )
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("Constructor Labs — Camera-Based Localization (Yas Marina)")
    print("=" * 60)
    print(f"Input MCAP: {args.mcap}")
    print(f"Output CSV: {args.out}")
    print("=" * 60)
    
    results = run_localization(
        mcap_path=args.mcap,
        output_csv_path=args.out,
        bounds_path=args.bounds
    )
    
    print("\n[Done] Pipeline finished successfully.")

if __name__ == "__main__":
    main()
