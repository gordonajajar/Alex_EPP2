# redis_utils.py
import os, redis, msgpack, zlib

REDIS_HOST = os.getenv("REDIS_HOST", "ceggas@10.229.218")
REDIS_PORT = int(os.getenv("REDIS_PORT", 6379))
r = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, decode_responses=False)

def r_publish(topic: str, payload: tuple):
    packed = msgpack.packb(payload, use_bin_type=True)
    # compress SLAM map (>250 kB) so we don't flood the wire
    if topic == "slam/mappose":
        packed = zlib.compress(packed, level=1)
    r.publish(topic, packed)

def r_subscribe(topics: list[str]):
    ps = r.pubsub(ignore_subscribe_messages=True)
    ps.subscribe(*topics)
    return ps
