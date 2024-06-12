# SwissCat-on_robot

For the github action workflow, we want to build and push on our linux, and then we want the RPi to be able to pull the image. Since the linux architecture is amd64 and the RPi one is arm64, we need to use a docker container, created as the following:

```
docker buildx create   --name em_onrobot   --driver=docker-container

docker buildx inspect --bootstrap local

docker buildx build --platform linux/amd64,linux/arm64 --builder em_onrobot --output type=registry -t jcswisscat/em_rpi:latest .

```
