#!/usr/bin/env bash

if test -f /conf/config.yml; then
  cp /conf/config.yml /vroom-express/config.yml
else
  cp /vroom-express/config.yml /conf/config.yml
fi

if ! test -f /conf/access.log; then
  touch /conf/access.log
fi

cd /vroom-express && VROOM_ROUTER=${VROOM_ROUTER} VROOM_LOG=${VROOM_LOG} exec npm start
