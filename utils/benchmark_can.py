import can
import time
import sys
import threading

def send_command(bus, can_id, data, receiver_id):
    message = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    total_response_time = 0
    successful_responses = 0
    failures = 0

    try:
        # Send the message
        bus.send(message)

        # Wait for the response
        response = None
        start_wait = time.time()
        while (time.time() - start_wait) < 1.0:  # 1 second timeout for receiving
            response = bus.recv(timeout=0.1)  # Small timeout to check for response
            if response and response.arbitration_id == receiver_id:
                response_time = (time.time() - start_wait) * 1000  # Convert to milliseconds
                total_response_time += response_time
                successful_responses += 1
                break
        else:
            failures += 1
    except can.CanError as e:
        print(f"Message not sent: {e}")
        failures += 1

    return total_response_time, successful_responses, failures

def benchmark_can_write(channel, can_id, num_messages, command):
    # Configure CAN bus
    bus = can.interface.Bus(channel=channel, interface='socketcan')

    # Calculate receiver CAN ID
    receiver_id = 0x240 + (can_id - 0x140)

    # Command to test
    data_message = [command, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

    # Start benchmark
    start_time = time.time()
    total_response_time = 0
    successful_responses = 0
    failures = 0

    for _ in range(num_messages):
        response_time, successes, fails = send_command(bus, can_id, data_message, receiver_id)
        total_response_time += response_time
        successful_responses += successes
        failures += fails

    # Calculate results for each CAN ID
    end_time = time.time()
    total_time = end_time - start_time
    messages_per_second = num_messages / total_time
    average_response_time = total_response_time / successful_responses if successful_responses > 0 else 0

    print(f"CAN ID 0x{can_id:03X}: Sent {num_messages} messages in {total_time:.4f} seconds")
    print(f"Speed: {messages_per_second:.2f} messages/second")
    print(f"Failures: {failures}")
    print(f"Average response time: {average_response_time:.2f} milliseconds")

    # Send the final message
    final_message = can.Message(arbitration_id=can_id, data=[0x80], is_extended_id=False)
    try:
        bus.send(final_message)
        print("Final message sent.")
    except can.CanError as e:
        print(f"Final message not sent: {e}")

def thread_function(channel, can_id, num_messages, command):
    benchmark_can_write(channel, can_id, num_messages, command)

if __name__ == "__main__":
    if len(sys.argv) < 5:
        print("Usage: python benchmark_can.py <channel> <num_messages> <command> <can_id1> [<can_id2> ... <can_idN>]")
        sys.exit(1)

    channel = sys.argv[1]
    num_messages = int(sys.argv[2])
    command = int(sys.argv[3], 16)
    can_ids = [int(can_id, 16) for can_id in sys.argv[4:]]

    threads = []

    for can_id in can_ids:
        thread = threading.Thread(target=thread_function, args=(channel, can_id, num_messages, command))
        threads.append(thread)
        thread.start()

    for thread in threads:
        thread.join()

    print("All threads have finished execution.")

