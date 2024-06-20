# builds with --no-cache to update the git informations, in case new infos were pushed (otherwise takes from docker cache)

docker build . --no-cache -t username/img_name:tag