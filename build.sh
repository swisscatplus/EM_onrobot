# builds with --no-cache to update the git informations, in case new infos were pushed (otherwise takes from docker cache)

docker build --no-cache --build-arg GIT_USER= --build-arg GIT_TOKEN= -t username/img_name:tag Desktop/EM_onrobot
