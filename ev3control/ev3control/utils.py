"""Code shared by both master and slave clients."""

MASTER_COMMANDS = "commands"
SLAVE_RESPONSES = "responses"


def decode_mqtt(msg):
    """Convenience function for decoding the payload of an MQTT message."""
    return msg.payload.decode()
