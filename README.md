# [IJCV 25] Image-based Morphological Characterization of Filamentous Biological Structures with Non-constant Curvature Shape Feature

This repository contains the implementation for **3D tendril-like shape reconstruction** from multi-view videos.  
The pipeline reconstructs a dynamic tendril model using three synchronized camera views, followed by curve fitting with piecewise clothoid models.

---

## 📥 Dataset Download

Due to large file size, the dataset is hosted externally.

🔗 **[Download Dataset from Google Drive](https://drive.google.com/drive/folders/19pg8yRdUplNgLFJYqhHsDnQVIhWlvjwM?usp=sharing)**

After downloading, place the entire dataset folder under: *Data/Stimulated_3rd_seg/*


## 🗂️ Project Structure
```
IJCV25-3d_Tendril-like_shape_reconstruction/
│
├── Data/
│ └── Stimulated_3rd_seg/
│ ├── view1.MOV
│ ├── view2.MOV
│ ├── view3.MOV
│ ├── CamIntrPara_view1.mat
│ ├── CamIntrPara_view2.mat
│ ├── CamIntrPara_view3.mat
│ ├── CamExtrPara_Corners_view1.csv
│ ├── CamExtrPara_Corners_view2.csv
│ └── CamExtrPara_Corners_view3.csv
│
├── Step1_Reconstruction.m
├── Step2_ClothoidFit.m
├── CalibraArucoMarker.py
│
└── FrechetDis/
│ ├── clean_duplicate.m
│ ├── DiscreteFrechetDist.m
│ ├── interparc.m
│
└── Homography/
│ ├── epipole.m
│ ├── homography.m
│ ├── parallax.m
│ ├── pcond.m
│
└── Ordering/
│ ├── arrow.m
│ ├── chooseDirct_2seg.m
│ ├── chooseDirct_3seg.m
│ ├── collect_neighbor_changingH_weight.m
│ ├── find_next_PN_index.m
│ ├── orderingskel_1cross2seg.m
│ ├── orderingskel_1cross3seg.m
│ ├── orderingskel_without_Cr.m
│
└── PWC/
│ ├── AutoSeg_BestPCDfitting.m
│ ├── CompareFitting.m
│ ├── downsampling.m
│ ├── error_fit.m
│ ├── FrenetFitting.m
│ ├── getFittedCurvatureTorsion.m
│ ├── HeightLineFit.m
│ ├── powersmooth2.m
│ ├── prepare_ksegments_var.m
│ ├── recurseThroughWalkMatrix.m
│ ├── rigid_transform_3D.m
│ ├── TNB.m
│
└── Segment/
│ ├── segmentVideoFrame.m
│
└── ShowTogether/
│ ├── myNormalize.m
│ ├── natsort.m
│ ├── natsortfiles.m
│
└── Thinning/
│ ├── cleanedgelist.m
│ ├── edgecolor.m
│ ├── edgelink.m
│ ├── findendsjunctions.m
│ ├── Skeletonization.m
│ ├── thinningVideoFrame.m
│
└──LICENSE
└──README.md
```

---

## 📂 Data Description

### 1. Videos
Add the corresponding `.MOV` video files for each camera view under the correct stimulated segment folder.  
Each folder represents the stimulus performed in one specific segment (e.g., *Stimulated_3rd_seg*).

> Example: `Data/Stimulated_3rd_seg/view1.MOV`, `view2.MOV`, `view3.MOV`

---

### 2. Camera Intrinsic Parameters
Include **three intrinsic parameter files** (`CamIntrPara*.mat`) calculated using MATLAB’s **Camera Calibrator** Add-On.

> Example:  
> `CamIntrPara_view1.mat`, `CamIntrPara_view2.mat`, `CamIntrPara_view3.mat`

---

### 3. Camera Extrinsic Parameters
Include **three extrinsic parameter files** (`CamExtrPara_Corners*.csv`) generated using the Python script `CalibraArucoMarker.py`.

Each CSV file contains per-frame data organized as follows:

| Column Range | Description |
|---------------|-------------|
| 1             | Frame index |
| 2             | ArUco marker index |
| 3–10          | Four ArUco corner coordinates (clockwise from top-left) |
| 11–13         | Translation vector (1×3) |
| 14–22         | Rotation matrix (1×9) |

> Each row corresponds to one frame in the video.

---

## 🚀 Reconstruction Pipeline

Once data are prepared, follow these two main steps.

### **Step 1: Multi-View 3D Reconstruction**

Reconstruct the tendril from three camera views and visualize it as a 3D point cloud.

1. Load video data and camera parameters.  
2. Semi-automatically segment and thin the target tendril in each image.  
3. Compute homography and epipolar geometry.  
4. Order the point cloud according to the tendril’s natural growth direction:
   - **Planar case:** without intersection points  
   - **Non-planar case:** with one intersection point  
5. Transform views 1 and 3 into the coordinate system of view 2.  
6. Register and find correspondences using **Fréchet distance**.  
7. Perform **multi-view triangulation**.  
8. Save each reconstructed frame.

---

### **Step 2: Piecewise Clothoid Curve Fitting**

Fit the reconstructed 3D tendril points using **3d piecewise clothoid curves** to achieve the smooth geometric continuity of curvature and torsion.

---

## 🧠 Notes

- The dataset folder structure must match the expected layout under `Data/Stimulated_3rd_seg/`.
- All MATLAB and Python scripts assume consistent naming across views.
- The example dataset corresponds to the **3rd stimulated segment** used in the paper.

---

## 📧 Contact

For questions or collaborations, please contact:  
**Jie Fan** — [GitHub: jiefan0414](https://github.com/jiefan0414)

---



