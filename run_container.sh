sudo docker exec -it $(sudo docker ps | awk -F ' ' 'FNR == 2 {print $1}') bash
