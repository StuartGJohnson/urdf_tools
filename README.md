# urdf_tools
A collection (>=1) of tools for handy xacro/urdf doings. 

## xacro2mesh

```xacro2mesh.py``` converts a robot xacro file to a to glb/gltf/stl/obj file for viewing in ... various ways. This is particularly useful when writing up a robot project! This tool has not been tested on robot xacro/urdf which contains links to mesh (.stl) files - my initial robot models are entirely constructed from geometric primitives. 

## xacro2stl_cq

```xacro2stl_cq.py``` converts a robot xacro file to a .stl file for viewing directly in github, but via the cadquery python package. GPT5 had reasons for doing this (see the section on Credits / LLM Story below). This is particularly useful when writing up a robot project! This tool has not been tested on robot xacro/urdf  which contains links to mesh (.stl) files - my initial robot models are entirely constructed from geometric primitives. 

## Installation

If you only wish to run ```xacro2mesh.py```, then you can simply pip install via requirements.txt, using a conda environment or venv based on python 3.10:

```pip install -r requirements.txt```

If you want a cadquery-generated STL via ```xacro2stl_cq.py```, cadquery installation can be quite a challenge. I suggest using mamba (install this), and performing the following incantation to obtain a usable conda env:

```mamba env create -f environment.yml```

### Installation: personal issues (only applies to ```xacro2stl_cq.py```)
Note that I had a particular challenge with mamba installed in my system python. This was due to issues with an upgrade from Ubuntu 20.04 to Ubuntu 22.04. I had to cook up a clean conda env/mamba install (thanks GPT4o for the suggestion) which was called ```py310tools```. And so my install incantation was:

```conda run -n py310tools mamba env create -f environment.yml```

I then activated this conda env prior to usage:

```conda activate urdf_cq```

## Usage

The python scripts can digest xacro or urdf files. 

```python xacro2mesh.py ugv.xacro -o ugv_x.stl --format stl```

```python xacro2mesh.py ugv.xacro -o ugv.glb --glb-convention gltf```

or via the cadquery implementation:

```python xacro2stl_cq.py ugv.xacro -o ugv.stl```


The .stl file can be linked to your README.md:

[▶️ View the robot in 3D](./ugv.stl)

The glb file can be viewed in github pages. For example:
[▶️ View the robot in 3D](https://stuartgjohnson.github.io/urdf_tools/)

Voila!

## Credits/LLM story

This code was developed from my concept via GPT5 (OpenAI, August-September 2025). This progressed as follows:

- I suggested to GPT5 that it would be useful to have a tool which converted xacro robot files to something viewable in github. I suggested perhaps an .stl file (via the python package cadquery), since I have had some luck with that. I first asked for comments.
- GPT5 suggested using urdfpy and generating a .glb file. This would retain the appearance of the robot as specified in the xacro file. Other suggestions included using blender. But, I only resort to GUIs when absolutely necessary or appropriate. I asked GPT5 to generate the urdfpy-based code. It turns out GPT5 was in error about .glb file viewing being supported by github: github does not support .glb file viewing. It is critical to have GPT5 check its assumptions at this phase of a project!
- I attempted to pip install urdfpy, but could never resolve conflicts in dependencies. I told GPT5 this.
- GPT5 suggested some install alternatives. I could not get these to work. I informed GPT5.
- GPT5 suggested using yourdfpy, which did pip install successfully.
- GPT5 was in error about yourdfpy usage (a typical LLM problem when confronted with coding against APIs/libraries/packages). I informed GPT5 of these issues with specific exceptions and line numbers (I was using pycharm and could also give good feedback to GPT5). After two iterations, GPT5 had the current code, which appears to work!
- I discover GPT5 was in error about a basic design assumption: in fact, github cannot view .glb files! It CAN view .stl files.
- I ask GPT5 to write a urdf to stl converter.
- GPT5 makes a minor error in cylinder rendering. I report the specifics of the error.
- GPT5 fixes the error.
- GPT5 and I circle on trying to get a javascript viewer working for the glb file. In the process of doing this, we discover that the glb/gltf standard coordinate system does not have z up. So GPT5 suggests a fix which we bake into xacro2mesh.py. In the process of doing this, GPT5 provides a cadquery-free (almost trivial) script which also generates stl files. I asked GPT5 why it was using cadquery in the first place. It mentioned: 1) I mentioned cadquery, and 2) cadquery generates better stl. Sigh.

IMHO: While LLMs can speed up development, the providers of these tools still have a mountain of work to do. AGI? Ha! Also, we have work to do to learn how to use these tools. 

### GPT5's prompting suggestions

After discovering that github cannot view .glb files, I asked GPT5 to tell me how to prompt it to avoid the situation (that is, GPT5 being in error about github file viewing capabilities). The trick is coming up with these early in your project! This is the recipe section it provided (GPT5 (OpenAI, September 2025)):

A quick prompting recipe

- Add a Feasibility Check step (mandatory).

  “Before writing any code, verify with web.run whether GitHub renders .glb inline. Use official GitHub docs as a primary source. If unsupported, propose viable alternatives. Do not produce code until you confirm.”

- State acceptance criteria explicitly.

  “Definition of done: I can click a link in README.md and see the model rendered in GitHub’s UI (no download, no external site). Provide a working snippet.”

- Require citations and a decision.

  “Cite the source that proves support/non-support. Then pick the best path (e.g., STL inline, or GLB via GitHub Pages + <model-viewer>), explain tradeoffs, and proceed.”

- List assumptions to confirm/deny.

  “Assumptions you must confirm: A) .glb is viewable inline; B) STL is viewable inline; C) Pages supports <model-viewer>.”


