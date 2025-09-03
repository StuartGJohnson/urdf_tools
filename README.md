# urdf_tools
A collection (>=1) of tools for handy xacro/urdf doings. 

## xacro2mesh

xacro2mesh.py converts a robot xacro file to a .glb file for viewing directly in github. This is particularly useful when writing up a robot project! At the time of writing, this was not tested on robots which contain links to meshes (.stl) files - my initial robot models are entirely constructed from geometric primitives.

### installation

I used a conda environment based on python 3.10, e.g.:

```conda create --name urdf python=3.10```

I then activated this conda env, and proceeded to pip away (from the directory of this repo):

```conda activate urdf```

```pip install -r requirements.txt```

### usage

The python script can digest xacro or urdf files:

```python xacro2mesh.py ugv.xacro -o ugv.glb```

or

```python xacro2mesh.py ugv.urdf -o ugv.glb```

The .glb file can be linked to your README.md:

[▶️ View the robot in 3D](./ugv.glb)

Voila!

### Credits

This code was developed from my concept via GPT5 (OpenAI, August-September 2025). This progressed as follows:

- I suggested to GPT5 that it would be useful to have a tool which converted xacro robot files to something viewable in github. I suggested perhaps an .stl file (via the python package cadquery), since I have had some luck with that. I first asked for comments.
- GPT5 suggested using urdfpy and generating a .glb file. This would retain the appearance of the robot as specified in the xacro file. Other suggestions included using blender. But, I only resort to GUIs when absolutely necessary or appropriate. I asked GPT5 to generate the urdfpy-based code.
- I attempted to pip install urdfpy, but could never resolve conflicts in dependencies. I told GPT5 this.
- GPT5 suggested some install alternatives. I could not get these to work. I informed GPT5.
- GPT5 suggested using yourdfpy, which did pip install successfully.
- GPT5 was in error about yourdfpy usage (a typical LLM problem when confronted with coding against APIs/libraries/packages). I informed GPT5 of these issues with specific exceptions and line numbers (I was using pycharm and could also give good feedback to GPT5). After two iterations, GPT5 had the current code, which appears to work!

IMHO: While LLMs can speed up development, the providers of these tools still have a mountain of work to do. Also, we have work to do to learn how to use these tools.
