# Computer Animation HW2 Report
###### tags: `CA2023`

### Introduction/Motivation
The objective of this homework is to learn **Forward Kinematics** and **Time Warping**. We are going to implement *forwardSolver()* and *timeWarper()* to finish the running/shooting ball animation.

<img src="https://i.imgur.com/BSVufMp.png" style="zoom:130%;" />![](https://i.imgur.com/sZnWMGW.png)

### Fundamentals 
The local position of bone is based on bone axis, so we need to calculate the global position of the bone. Furthermore, if we want to calculate the translation and roatation of i-th bone, we need to consider 1 to i-1 bone.  
     
### Implementation
* fowardSolver:
    ![](https://i.imgur.com/pS9Hoji.png)

     * traverse all bones:
         I use **DFS** traverse all the bones, i.e. visit the child first then the sibling.
         
     * calculate the *rotation*:
         For the first bone, the *roatation* = equal to the **"bone->rot_parent_current(=identity matrix at first)"** *      **"posture.bone_rotation[bone_idx]"**.
         But for the other bones, we also need to multiply the roatation of the paraent of the current bone. 
         
     * calculate the *start position*:
         For the first bone , we only need to consider the initialized translations.
         But for the other bones, we calculate the start position from the end position from the parent of teh current bone. 
         
     * calculate the *end position*:
     For the end position of the bone, we get it from the following formula.
         
        
        
        ![](https://i.imgur.com/VMGTI53.png)
    
* timeWarper:
    Time warping is to projetc the current number of frames to another number of the frames. (ex. 150 to 160 or 150 to 140) 
    * translation:
        We use linear interpolation to calculate the vlaue of new translations.
    * rotations:
        We use spherical linear interpolation. And we need to transfer the rotation vlaue to **Quaternoid** first, then calculate the new roations.
    
    ![](https://i.imgur.com/LiYUhHV.png)

### Result and Discussion
* Time warping on:
    <img src="https://i.imgur.com/g1rBEY2.png =50%x" style="zoom:50%;" /><img src="https://i.imgur.com/ctJKtVw.png =50%x" style="zoom:50%;" />
* Time warping off:
    * post keyframe = 100, at frame 43
        ![](https://i.imgur.com/MWTlai6.png)
    * post keyframe = 200, at frame 150
        ![](https://i.imgur.com/m6COrq2.png)
### Bonus (Optional)
### Conclusion
In this homework, I learnt (1) How to implement the foward linematics (2) How to implement time warping (3) The Eigen Library