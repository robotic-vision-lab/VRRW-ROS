# Robosim-Docker
Dockerfiles and scripts for robotics simulation with Unity and MoveIt! Motion Planning Framework.

![demo](./demo.gif)

## Usage

### 1.Clone the repository

```bash
git clone --recursive git@github.com:robotic-vision-lab/Robosim-Docker.git
```

### 2. Build the Docker Image

<!-- TODO: Create a Docker Hub repository so images can be pulled directly over the web instead of building. -->

```bash
# assuming the shell is in the repository directory
docker build -t robosim:latest -f ./dockerfiles/Dockerfile.noetic .
```

### 3. Create and Access the Container

```bash
# assuming the shell is in the repository directory
sh scripts/run_robosim_container.sh
```
