import os
import numpy as np
from matplotlib import pyplot as plt
from laserscan import LaserScan, SemLaserScan
debug = False

def main():
    label_dir = "caetec_bin/labels/"
    bin_dir = "caetec_bin/velodyne/"
    out_dir = "caetec_bin/"
    file_prefixes = [os.path.splitext(os.path.basename(f))[0]
                     for f in os.listdir(bin_dir)]
    for pref in file_prefixes:
        scan = SemLaserScan(project=True, H=16, fov_up=15.0, fov_down=-15.0)
        scan.open_scan(os.path.join(bin_dir, pref + ".bin"))
        scan.open_label(os.path.join(label_dir, pref + ".label"))
        if len(np.unique(scan.proj_sem_label)) > 1:
            label = scan.proj_sem_label[:, 1250:1762]
            xyz = scan.proj_xyz[:, 1250:1762]
            r = scan.proj_range[:, 1250:1762]  # range
            remission = scan.proj_remission[:, 1250:1762]  # same as intensity?
            res = np.zeros((16, 512, 6))
            res[:, :, :3] = xyz
            res[:, :, 3] = remission
            res[:, :, 4] = r
            res[:, :, 5] = label
            np.save(os.path.join(out_dir, pref), res)
            if debug:
                plt.subplot(2, 1, 1)
                plt.imshow(scan.proj_sem_label[:, 1250:1762])
                print(scan.proj_sem_label[:, 1250:1762].shape)
                # plt.imshow(scan.proj_sem_label)
                plt.subplot(2, 1, 2)
                plt.imshow(scan.proj_remission[:, 1250:1762])
                # plt.imshow(scan.proj_remission)

                plt.show()
                break


if __name__ == "__main__":
    main()
