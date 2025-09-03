# urdf_tools
A collection (>=1) of tools for handy xacro/urdf doings. 

## xacro2mesh

```xacro2mesh.py``` converts a robot xacro file to a .glb file for viewing in ... various ways. This is particularly useful when writing up a robot project! This tool has not been tested on robots which contain links to meshes (.stl) files - my initial robot models are entirely constructed from geometric primitives. While not directly viewable in github, this file does contain model appearance data (not just geometry).

```xacro2stl_cq.py``` converts a robot xacro file to a .stl file for viewing directly in github. This is particularly useful when writing up a robot project! This tool has not been tested on robots which contain links to meshes (.stl) files - my initial robot models are entirely constructed from geometric primitives. While directly viewable in github, this file does not contain model appearance data (just geometry).

### installation

cadquery installation can be quite a challenge. I suggest using mamba (install this), and performing the following incantation to obtain a usable conda env:

```mamba env create -f environment.yml```

#### personal issues
Note that I had a particular challenge with mamba installed in my system python. This was due to issues with an upgrade from Ubuntu 20.04 to Ubuntu 22.04. I had to cook up a clean conda env/mamba install (thanks GPT4o for the suggestion) which was called ```py310tools```. And so my install incantation was:

```conda run -n py310tools mamba env create -f environment.yml```

I then activated this conda env prior to usage:

```conda activate urdf-cq```


### usage

The python scripts can digest xacro or urdf files:

```python xacro2mesh.py ugv.xacro -o ugv.glb```

```python xacro2stl_cq.py ugv.xacro -o ugv.stl```

or

```python xacro2mesh.py ugv.urdf -o ugv.glb```

```python xacro2stl_cq.py ugv.urdf -o ugv.stl```

The .stl file can be linked to your README.md:

[▶️ View the robot in 3D](./ugv.stl)

The glb file can be viewed in github pages or in many 3d viewers. There will be a companion github page showing this.

Voila!

### Credits/LLM story

This code was developed from my concept via GPT5 (OpenAI, August-September 2025). This progressed as follows:

- I suggested to GPT5 that it would be useful to have a tool which converted xacro robot files to something viewable in github. I suggested perhaps an .stl file (via the python package cadquery), since I have had some luck with that. I first asked for comments.
- GPT5 suggested using urdfpy and generating a .glb file. This would retain the appearance of the robot as specified in the xacro file. Other suggestions included using blender. But, I only resort to GUIs when absolutely necessary or appropriate. I asked GPT5 to generate the urdfpy-based code.
- I attempted to pip install urdfpy, but could never resolve conflicts in dependencies. I told GPT5 this.
- GPT5 suggested some install alternatives. I could not get these to work. I informed GPT5.
- GPT5 suggested using yourdfpy, which did pip install successfully.
- GPT5 was in error about yourdfpy usage (a typical LLM problem when confronted with coding against APIs/libraries/packages). I informed GPT5 of these issues with specific exceptions and line numbers (I was using pycharm and could also give good feedback to GPT5). After two iterations, GPT5 had the current code, which appears to work!
- GPT5 was in error about a basic design assumption: in fact, github cannot view .glb files! It CAN view .stl files. It is critical to have GPT5 check its assumptions at this phase of a project!
- I ask GPT5 to write a urdf to stl converter.
- GPT5 makes a minor error in cylinder rendering. I report the specifics of the error.
- GPT5 fixes the error.

IMHO: While LLMs can speed up development, the providers of these tools still have a mountain of work to do. Also, we have work to do to learn how to use these tools.
