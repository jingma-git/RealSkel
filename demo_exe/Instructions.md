## ReadFirst
1. Many shortkeys are similar to those of the opensource software 'Blender'.
2. 'A subpart' means 'a leg', 'an arm', 'an ear' etc.
3. The coordinate system using OpenGL's coord, which is left-hand coordinate system with X pointing to right, Y pointing up, Z pointing out of screen.
4. Screenshot: press F2, the rendered image will be saved in the 'screenshot' directory.
5. Save Project: press Ctrl+S, in the pop up window, create/select a directory to save your project file. The system automatically save the 'project file' under 'output' directory, if the system crashes, you can load project from 'output' directory and continue your operation.
6. Load Project: press Ctrl+L, in the pop up window, please choose the directory where you saves your project.

## Views

### Split View
1. Two views: click 'View'-->'2D & 3D Views' (left sub-window is 2D view, right sub-window is 3D view)
2. Rearrange views: adjust window size after resizing the window

### Change View
***Remember to select a sub-window before the following operations!!!***
1. Change view for the sub-window
   1. Front View: numpad 1 (Rightside Keyboard)
   2. Back View: ctrl + numpad 1
   3. Right View: numpad 3
   4. Left View: ctrl + numpad 3
   5. Top View: numpad 7
   6. Bottom View: ctrl + numpad 7
   7. Perspective View: drag with the mouse-middle-button.
2. Toggle 2D&3D View: Press 'Backslash'.
3. Zoom-in/out: Mouse Wheel.

## Background Image
***Remember to select a sub-window before the following operations!!!***
1. Open the background image:  Ctrl + O.
2. Scale the background image: Ctrl + Mouse Wheel.
3. Translate the background image: Ctrl + Drag-with-Mouse-Left-Button.

## Shape & Skeleton creation

1. Create the initial shape&skeleton: Press 'Space' to switch to ***Sketch Mode***. The shape&skeleton will be automatically generated after the user finishes drawing.
2. Select
   1. Press 'S' to switch to the ***Select Mode***.
   2. Click the subpart. The selected subpart will become darker. 
   3. ***Before 'Create symmetric component', 'Move', 'Rotate', 'Scale', 'Adjust Thickness' and 'Modify Shape Contour', it is better to press 'S' to switch to the 'Select Mode' and select the subpart of interest. Otherwise, the system will use the default subpart (the default subpart is the newly created subpart or the subpart you picked last time).***
3. Create symmetric component: Ctrl+M
4. Move 
   1. Switch to the 'Translation Mode': Press 'G'.
   2. Drag with right-click.
5. Rotate
   1. Switch to the 'Rotation Mode': Press 'R'. The rotating center will show.
   2. Press 'X', 'Y', 'Z' to select the rotate axis.
   3. Drag with left-click.
6. Scale
   1. Switch to the ***Select Mode***.
   2. ***Double clicking*** the subpart pops up a dialog.
   3. Adjust the 'scale' slide of the pop-up dialog.
7. Adjust Depth
   1. Switch to the ***Select Mode***.
   2. ***Double clicking** the subpart pops up a dialog.
   3. Adjust the 'depth' slide of the pop-up dialog, there is 'X', 'Y', 'Z' radio button above the 'depth' slide, which means put the compoent on '*' depth under 'X'/'Y'/'Z' axis.
8. Adjust Thickness
   1. Switch to the ***Select Mode***.
   2. ***Double clicking** the subpart pops up a dialog.
   3. Adjust the 'thickness' slide of the pop-up dialog.
9.  Modify the Shape Contour
   4. Switch to 'Contour Modifying Mode': Press 'M'. The system will show the contour with control points.
   5. Pick the control point and drag.

## Skeleton Refinement
***Remember to Press 'S' to switch to the 'Select Mode'***
### Step1: Selection
1. Single selection: left-click.
2. Multiple selection: Ctrl + left-click
3. Select all: Ctrl + A
4. Unselect all: Alt + A
### Step2: Refine the skeleton on the selected part
1. Pressing 'C' pops up a dialog.
2. Try to move the slide to ajdust bone complexity
   1. Branch simplification: reduce/increase number of bones for a branch.
   2. Merging: merge junction joints (joint with more than three adjacent bones).
   3. Trimming: trim the terminal joint (joint on the branch terminal, and the bone is very short).
   4. Collapsing: collapse short bones.

## Deformation
1. Press 'D' to switch to ***Deformation Mode***. The system will automatically bind the bone to the skin/mesh. 
2. When there is a patch of mesh become red (the influence of the joint to the mesh), the binding finishes.
3. Pick the joint and drag with 'right-click'.

## Other Functions
1. Toggle Transparency: Press 'T'.
2. Toggle Wireframe Rendering: Press 'P'.
3. Toggle Depth Test: Press 'L'.
   ```
   See the Skeleton: set Transparency as True, set DepthTest as False; or render with Wireframe.
   ```
4. Toggle skinning weight: Press 'Backspace'. Then press 'Period' to see the skinning weight for different joint.