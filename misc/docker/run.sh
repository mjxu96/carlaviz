# set up frontend
service nginx restart

# start backend
./backend/bin/backend $@
