import asyncio
from typing import TYPE_CHECKING

import can

if TYPE_CHECKING:
    from can.notifier import MessageRecipient


def print_message(msg: can.Message) -> None:
    """Regular callback function. Can also be a coroutine."""
    print(msg)


async def main() -> None:
    """The main function that runs in the loop."""

    with can.Bus(interface="can0socketcan", channel="can0", receive_own_messages=True) as bus:
        reader = can.AsyncBufferedReader()
        logger = can.Logger("logfile.asc")
    msg = await reader.get_message()
    print(msg)


if __name__ == "__main__":
    asyncio.run(main())