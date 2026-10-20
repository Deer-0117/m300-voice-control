#!/usr/bin/env python3
"""Create package-local references to the existing Liveview producer. No service changes."""
import argparse,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))

def main():
    p=argparse.ArgumentParser();p.add_argument('--metadata',required=True,type=Path);p.add_argument('--frame-dir',required=True,type=Path);p.add_argument('--fifo',required=True,type=Path);a=p.parse_args()
    dest=ROOT/'inputs';dest.mkdir(exist_ok=True)
    for name,target in [('liveview_latest.tsv',a.metadata),('liveview_frames',a.frame_dir),('liveview.h264',a.fifo)]:
        link=dest/name
        if link.is_symlink():link.unlink()
        elif link.exists():raise SystemExit('Refusing to replace existing non-symlink: '+str(link))
        link.symlink_to(target.expanduser().resolve(),target_is_directory=name=='liveview_frames')
    print('Created inputs references. No decoder/PSDK service was started or stopped.')
if __name__=='__main__':main()
