#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Song Jin
# 画仿真实验的图，8张图放两排，每个位置画一张
import os
import subprocess
import argparse
from datetime import datetime
from shutil import copyfile

import numpy as np
from colorama import init, Fore
import matplotlib.pyplot as plt
from matplotlib import rc
from mpl_toolkits.mplot3d import Axes3D

import exp_utils as eu

import matplotlib

print(matplotlib.__version__)

_save_ext = ".pdf"

rc('font', **{'family': 'serif', 'serif': ['Cardo'], 'size': 20})
# rc('text', usetex=True)


def normalize(x):
    vmax = np.max(x)
    vmin = np.min(x)
    return np.array([(v - vmin)/(vmax-vmin) for v in x.tolist()])


def plotSingle(ax, vos_pos, points, plt_rng, res, nm):
    ax.scatter(points[0:-1:3, 0], points[0:-1:3, 1], points[0:-1:3, 2])
    ax.set_xlim(-plt_rng/2, plt_rng/2)
    ax.set_ylim(-plt_rng/2, plt_rng/2)
    ax.set_zlim(-plt_rng/2, plt_rng/2)
    colors = [[0, v, 0] for v in res['nvalues']]
    arrow_colors = colors + [v for v in colors for _ in (0, 1)]
    # arrow_colors = colors + [[1, 0, 1] for v in colors for _ in (0, 1)]
    ax.quiver(vox_pos[:, 0], vox_pos[:, 1], vox_pos[:, 2],
              res['views'][:, 0], res['views'][:, 1], res['views'][:, 2],
              length=0.3, color=arrow_colors,
              arrow_length_ratio=0.7)
    ax.set_title(nm)
    ax.view_init(azim=0, elev=90)


def plotAllinSingel(ax, vos_pos, index, map_points, view_points, plt_rng, res, nm):
    #ax.scatter(map_points[0:-1:3, 0], map_points[0:-1:3, 1], map_points[0:-1:3, 2], c='blue')
    ax.scatter(view_points[0:-1:3, 0], view_points[0:-1:3, 1], view_points[0:-1:3, 2], c='red')
    ax.set_xlim(-plt_rng/2, plt_rng/2)
    ax.set_ylim(-plt_rng/2, plt_rng/2)
    ax.set_zlim(-plt_rng/2, plt_rng/2)
    # colors = [[0, v, 0] for v in res['nvalues']]
    # arrow_colors = colors + [v for v in colors for _ in (0, 1)]
    # arrow_colors = colors + [[1, 0, 1] for v in colors for _ in (0, 1)]
    ax.quiver(vox_pos[index, 0], vox_pos[index, 1], vox_pos[index, 2],
              res['trace']['views'][index, 0], res['trace']['views'][index, 1], res['trace']['views'][index, 2],
              length=1, color='red', pivot='tail',
              arrow_length_ratio=0.7)
    ax.quiver(vox_pos[index, 0], vox_pos[index, 1], vox_pos[index, 2],
              res['det']['views'][index, 0], res['det']['views'][index, 1], res['det']['views'][index, 2],
              length=1, color='green',pivot='tail',
              arrow_length_ratio=0.7)
    ax.quiver(vox_pos[index, 0], vox_pos[index, 1], vox_pos[index, 2],
              res['mineig']['views'][index, 0], res['mineig']['views'][index, 1], res['mineig']['views'][index, 2],
              length=1, color='blue',pivot='tail',
              arrow_length_ratio=0.7)    
    # plot circle
    theta = np.linspace(-np.pi/2, np.pi/2, 201)
    x = 1.35*np.cos(theta)
    y = 1.35*np.sin(theta)          
    ax.plot(x, y, color='black',label="trajectory")       

    ax.scatter(vox_pos[index, 0], vox_pos[index, 1], vox_pos[index, 2], s=80, marker='^')                 
    ax.set_title(nm)
    ax.view_init(azim=0, elev=90)

def plotAll(ax, vos_pos, map_points, plt_rng, res, nm):
    ax.scatter(map_points[0:-1:3, 0], map_points[0:-1:3, 1], map_points[0:-1:3, 2], c='blue')
    ax.set_xlim(-plt_rng/2, plt_rng/2)
    ax.set_ylim(-plt_rng/2, plt_rng/2)
    ax.set_zlim(-plt_rng/2, plt_rng/2)
    # colors = [[0, v, 0] for v in res['nvalues']]
    # arrow_colors = colors + [v for v in colors for _ in (0, 1)]
    # arrow_colors = colors + [[1, 0, 1] for v in colors for _ in (0, 1)]
    ax.quiver(vox_pos[:, 0], vox_pos[:, 1], vox_pos[:, 2],
              res['trace']['views'][:, 0], res['trace']['views'][:, 1], res['trace']['views'][:, 2],
              length=0.5, color='red',pivot='tail',
              arrow_length_ratio=0.7)
    ax.quiver(vox_pos[:, 0], vox_pos[:, 1], vox_pos[:, 2],
              res['det']['views'][:, 0], res['det']['views'][:, 1], res['det']['views'][:, 2],
              length=0.5, color='green',pivot='tail',
              arrow_length_ratio=0.7)
    ax.quiver(vox_pos[:, 0], vox_pos[:, 1], vox_pos[:, 2],
              res['mineig']['views'][:, 0], res['mineig']['views'][:, 1], res['mineig']['views'][:, 2],
              length=0.5, color='blue',pivot='tail',
              arrow_length_ratio=0.7)   
    # plot circle
    theta = np.linspace(-np.pi/2, np.pi/2, 201)
    x = 1.35*np.cos(theta)
    y = 1.35*np.sin(theta)          
    ax.plot(x, y, color='black',label="trajectory")
    ax.scatter(vox_pos[:, 0], vox_pos[:, 1], vox_pos[:, 2], s=80, marker='^')  
    ax.set_title(nm)
    ax.view_init(azim=0, elev=90)

def plotErrMap(ax, vox_pos, plt_rng, err, nm):
    ax.set_xlim(-plt_rng/2, plt_rng/2)
    ax.set_ylim(-plt_rng/2, plt_rng/2)
    ax.set_zlim(-plt_rng/2, plt_rng/2)
    n_err = normalize(np.array(err))
    colors = np.array([[v, 0, 0] for v in n_err])
    ax.scatter(vox_pos[:, 0], vox_pos[:, 1], vox_pos[:, 2],
               c=colors)
    ax.set_title(nm)
    ax.view_init(azim=0, elev=90)


init(autoreset=True)
_save_ext = ".pdf"
rc('font', **{'family': 'serif', 'serif': ['Cardo'], 'size': 20})
# rc('text', usetex=True)

parser = argparse.ArgumentParser()

parser.add_argument('--xrange', default=4.0, type=float,
                    help='Zero centered x range.')
parser.add_argument('--yrange', default=4.0, type=float,
                    help='Zero centered y range.')
parser.add_argument('--zrange', default=2.0, type=float,
                    help='Zero centered z range.')
parser.add_argument('--animated', action='store_true',
                    help='whether to visualize it in animation')
parser.add_argument('--dt', default=0.01, type=float,
                    help='animation delta t')
args = parser.parse_args()
print(args)


fn_base = "exp_best_orient_nrsl"
act_map = "/home/user/Project/Final_project/amphibious_robot_ws/src/information_filed/rpg_information_field/act_map"
top_trace_dir = os.path.join(act_map,
                             "trace/"+fn_base)
abs_trace_dir = os.path.join(top_trace_dir,"four_walls_bestView")

print(Fore.RED + ">>>>> Start analysis.")

test_mtypes = ['trace', 'det', 'mineig']
ceres = 'ceres_optimization'
ext = '.txt'
view_pre = 'optim_view_'
val_pre = 'optim_value_'

vox_pos = np.loadtxt(abs_trace_dir + "/vox_pos" + ext)
map_points = np.loadtxt(abs_trace_dir + "/map_points" + ext)

points0 = np.loadtxt(abs_trace_dir + "/vis_point0" + ext)
points1 = np.loadtxt(abs_trace_dir + "/vis_point1" + ext)
points2 = np.loadtxt(abs_trace_dir + "/vis_point2" + ext)
points3 = np.loadtxt(abs_trace_dir + "/vis_point3" + ext)
points4 = np.loadtxt(abs_trace_dir + "/vis_point4" + ext)
points5 = np.loadtxt(abs_trace_dir + "/vis_point5" + ext)
points6 = np.loadtxt(abs_trace_dir + "/vis_point6" + ext)

gt_res = {}
app_res = {}
for mtype in test_mtypes:
    gt_nm_i = mtype + '_' + ceres
    gt_res_i = {}
    gt_res_i['views'] = np.loadtxt(os.path.join(abs_trace_dir,
                                                view_pre+gt_nm_i+ext))
    gt_res_i['values'] = np.loadtxt(os.path.join(abs_trace_dir,
                                                 val_pre+gt_nm_i+ext))
    gt_res[mtype] = gt_res_i

gt_res['trace']['nvalues'] =\
    normalize(np.log(gt_res['trace']['values']))

gt_res['det']['nvalues'] =\
    normalize(np.log(gt_res['det']['values']))

gt_res['mineig']['nvalues'] =\
    normalize(np.log(gt_res['mineig']['values']))

print(gt_res['mineig']['views'])
### paper plots

# fig_save_trace_app = plt.figure(figsize=(6, 6))
# fig_save_mineig_app = plt.figure(figsize=(6, 6))
# fig_save_det_app = plt.figure(figsize=(6, 6))

# fig_save_trace_exa = plt.figure(figsize=(6, 6))
# fig_save_mineig_exa = plt.figure(figsize=(6, 6))
# fig_save_det_exa = plt.figure(figsize=(6, 6))

# ax = fig_save_mineig_app.add_subplot(111, projection='3d')
# plotSingle(ax, vox_pos, points, args.xrange, app_res['mineig'],
           # r'$\sigma_{min}$ poly.')
# plt.tight_layout()
# fig_save_mineig_app.savefig(plot_dirs + '/mineigh_comp_app' + _save_ext, bbox_inches='tight')
# ax = fig_save_mineig_exa.add_subplot(111, projection='3d')
# plotSingle(ax, vox_pos, points, args.xrange, gt_res['mineig'],
           # r'$\sigma_{min}$')
# plt.tight_layout()
# fig_save_mineig_exa.savefig(plot_dirs + '/mineigh_comp_exa' + _save_ext, bbox_inches='tight')


# ax = fig_save_trace_app.add_subplot(111, projection='3d')
# plotSingle(ax, vox_pos, points, args.xrange, app_res['trace'],
           # r'Trace poly.')
# plt.tight_layout()
# fig_save_trace_app.savefig(plot_dirs + '/trace_comp_app' + _save_ext, bbox_inches='tight')
# ax = fig_save_trace_exa.add_subplot(111, projection='3d')
# plotSingle(ax, vox_pos, points, args.xrange, gt_res['trace'],
           # r'Trace')
# plt.tight_layout()
# fig_save_trace_exa.savefig(plot_dirs + '/trace_comp_exa' + _save_ext, bbox_inches='tight')


# ax = fig_save_det_exa.add_subplot(111, projection='3d')
# plotSingle(ax, vox_pos, points, args.xrange, app_res['det'],
           # r'Det poly.')
# plt.tight_layout()
# fig_save_det_exa.savefig(plot_dirs + '/det_comp_exa' + _save_ext, bbox_inches='tight')
# ax = fig_save_det_app.add_subplot(111, projection='3d')
# plotSingle(ax, vox_pos, points, args.xrange, gt_res['det'],
           # r'Det')
# plt.tight_layout()
# fig_save_det_app.savefig(plot_dirs + '/det_comp_app' + _save_ext, bbox_inches='tight')


### visualiation for compare
axes = []
fig = plt.figure(figsize=(27, 14))

ax = fig.add_subplot(241, projection='3d')
plotAllinSingel(ax, vox_pos, 0, map_points, points0, args.xrange, gt_res,
           r'Position 1')
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_zticklabels([])
axes.append(ax)

ax = fig.add_subplot(242, projection='3d')
plotAllinSingel(ax, vox_pos, 1, map_points, points1, args.xrange, gt_res,
           r'Position 2')
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_zticklabels([])
axes.append(ax)

ax = fig.add_subplot(243, projection='3d')
plotAllinSingel(ax, vox_pos, 2, map_points, points2, args.xrange, gt_res,
           r'Position 3')
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_zticklabels([])
axes.append(ax)

ax = fig.add_subplot(244, projection='3d')
plotAllinSingel(ax, vox_pos, 3, map_points, points3, args.xrange, gt_res,
           r'Position 4')
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_zticklabels([])
axes.append(ax)

ax = fig.add_subplot(245, projection='3d')
plotAllinSingel(ax, vox_pos, 4, map_points, points4, args.xrange, gt_res,
           r'Position 5')
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_zticklabels([])
axes.append(ax)

ax = fig.add_subplot(246, projection='3d')
plotAllinSingel(ax, vox_pos, 5, map_points, points5, args.xrange, gt_res,
           r'Position 6')
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_zticklabels([])
axes.append(ax)

ax = fig.add_subplot(247, projection='3d')
plotAllinSingel(ax, vox_pos, 6, map_points, points6, args.xrange, gt_res,
           r'Position 7')
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_zticklabels([])
axes.append(ax)

ax = fig.add_subplot(248, projection='3d')
plotAll(ax, vox_pos, map_points, args.xrange, gt_res,
           r'All Position')
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_zticklabels([])
axes.append(ax)



plt.tight_layout()
if args.animated:
    azim = np.arange(0, 360, 2)

    p_elev = np.arange(50, 75, 0.5)
    n_elev = np.flip(p_elev)
    elev = np.hstack((p_elev, n_elev))
    n_elev = elev.shape[0]

    fixed_elev = 60
    for i in range(azim.shape[0]):
        a = azim[i]
        e = elev[i % n_elev]
        # print("Azim {0} and elev {1}".format(a, e))
        for ax in axes:
            ax.view_init(azim=a, elev=fixed_elev)
        plt.ion()
        plt.show()
        plt.pause(args.dt)
else:
    plt.show()

# ax = fig.add_subplot(337, projection='3d')
# plotErrMap(ax, vox_pos, args.xrange, err_min_eig, 'mineig error')

# ax = fig.add_subplot(338, projection='3d')
# plotErrMap(ax, vox_pos, args.xrange, err_trace, 'trace error')

# ax = fig.add_subplot(339, projection='3d')
# plotErrMap(ax, vox_pos, args.xrange, err_det, 'det error')

# fig = plt.figure()

# ax = fig.add_subplot(231)
# plt.hist(err_min_eig, bins=eu.hist_bins)
# ax.set_title('mineig: appr vs exact')

# ax = fig.add_subplot(232)
# plt.hist(err_trace, bins=eu.hist_bins)
# ax.set_title('trace: appr vs exact')

# ax = fig.add_subplot(233)
# plt.hist(err_det, bins=eu.hist_bins)
# ax.set_title('det: appr vs exact')

# ax = fig.add_subplot(234)
# plt.hist(err_diff_app, bins=eu.hist_bins)
# ax.set_title('appr: trace vs mineig')

# ax = fig.add_subplot(235)
# plt.hist(err_diff_exact1, bins=eu.hist_bins)
# ax.set_title('exact: trace vs mineig')

# ax = fig.add_subplot(236)
# plt.hist(err_diff_exact2, bins=eu.hist_bins)
# ax.set_title('exact: trace vs det')

## specific compare on the trace
# fig = plt.figure()
# ax = fig.add_subplot(221, projection='3d')
# plotSingle(ax, vox_pos, points, args.xrange, gt_res['trace'],
           # 'trace exact')
# ax = fig.add_subplot(222, projection='3d')
# plotSingle(ax, vox_pos, points, args.xrange, app_res['trace'],
           # 'trace approx')
# ax = fig.add_subplot(223, projection='3d')
# plotSingle(ax, vox_pos, points, args.xrange, closed_trace_res,
           # 'appr trace zero deriv')
# ax = fig.add_subplot(224, projection='3d')
# plotSingle(ax, vox_pos, points, args.xrange, worst_trace_res,
           # 'appr trace worst')
# plt.show()


print(Fore.GREEN + "<<<<< Analysis done.")

