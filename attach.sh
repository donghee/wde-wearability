docker exec -w $(pwd) -it $(docker ps | grep user_$USER | awk '{ print $17}') bash
