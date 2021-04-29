import numpy as np 
import matplotlib.pyplot as plt 

poses_gt = [np.vstack([x.reshape(3,4), np.array([[0,0,0,1]])]) for x in np.loadtxt("pose_gt.txt")][100:160]
pose0_inv = np.linalg.inv(poses_gt[0])
poses_gt = [pose0_inv.dot(x) for x in poses_gt]
tvecs_gt = np.array([x[:3, 3] for x in poses_gt])

tvecs_fr = np.loadtxt("pose_frontend_pr.txt")

tvecs_pr = np.loadtxt("pose_slam_pr.txt")
tvecs_pr_kf = tvecs_pr[tvecs_pr[:, 3] == 1.0]

# print('Gt\n', tvecs_gt[:tvecs_pr.shape[0]])
# print('Pr\n', tvecs_pr)

plt.plot(\
    tvecs_gt[:, 0], tvecs_gt[:, 2], "b-+", \
    tvecs_fr[:, 0], tvecs_fr[:, 2], "g^", \
    tvecs_pr[:, 0], tvecs_pr[:, 2], "r.", \
    tvecs_pr_kf[:, 0], tvecs_pr_kf[:, 2], "rs" \
)
fig = plt.gcf()
fig.savefig('plot.png', transparent=True, dpi=300)
plt.show()

