#!/usr/bin/env python3
"""
ROS Bag to CSV Converter
Converts rosbag files containing /msp/* topics to CSV format for analysis
"""

import rosbag
import csv
import sys
import os
from pathlib import Path


def extract_topic_to_csv(bag_file, topic_name, output_dir):
    """Extract a single topic from bag file to CSV"""

    # Create output filename
    topic_safe_name = topic_name.replace('/', '_').strip('_')
    csv_filename = os.path.join(output_dir, f"{topic_safe_name}.csv")

    # Open bag and collect messages
    messages = []
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            msg_dict = {'timestamp': t.to_sec()}

            # Extract message fields recursively
            extract_fields(msg, msg_dict)
            messages.append(msg_dict)

    if not messages:
        print(f"  No messages found for topic: {topic_name}")
        return

    # Write to CSV
    with open(csv_filename, 'w', newline='') as csvfile:
        fieldnames = messages[0].keys()
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(messages)

    print(f"  Exported {len(messages)} messages to: {csv_filename}")


def extract_fields(msg, result_dict, prefix=''):
    """Recursively extract fields from ROS message"""

    # Get all slots (fields) of the message
    if hasattr(msg, '__slots__'):
        for slot in msg.__slots__:
            value = getattr(msg, slot)
            field_name = f"{prefix}{slot}" if prefix else slot

            # Handle nested messages
            if hasattr(value, '__slots__'):
                extract_fields(value, result_dict, f"{field_name}.")
            # Handle lists/arrays
            elif isinstance(value, (list, tuple)):
                if len(value) > 0 and hasattr(value[0], '__slots__'):
                    # List of messages
                    for i, item in enumerate(value):
                        extract_fields(item, result_dict, f"{field_name}[{i}].")
                else:
                    # List of primitives
                    for i, item in enumerate(value):
                        result_dict[f"{field_name}[{i}]"] = item
            else:
                # Primitive type
                result_dict[field_name] = value


def convert_bag_to_csv(bag_file, output_dir=None):
    """Convert all /msp/* topics in a bag file to CSV files"""

    if not os.path.exists(bag_file):
        print(f"Error: Bag file not found: {bag_file}")
        return

    # Create output directory
    if output_dir is None:
        bag_name = Path(bag_file).stem
        output_dir = f"{bag_name}_csv"

    os.makedirs(output_dir, exist_ok=True)
    print(f"Converting bag file: {bag_file}")
    print(f"Output directory: {output_dir}")

    # Get list of /msp/* topics
    topics = []
    with rosbag.Bag(bag_file, 'r') as bag:
        info = bag.get_type_and_topic_info()
        topics = [t for t in info.topics.keys() if t.startswith('/msp/')]

    if not topics:
        print("No /msp/* topics found in bag file")
        return

    print(f"Found {len(topics)} /msp/* topics:")
    for topic in topics:
        print(f"  - {topic}")

    # Convert each topic
    print("\nConverting topics...")
    for topic in topics:
        extract_topic_to_csv(bag_file, topic, output_dir)

    print(f"\nConversion complete! CSV files saved to: {output_dir}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 bag_to_csv.py <bag_file> [output_dir]")
        print("\nExample:")
        print("  python3 bag_to_csv.py /data/rosbag/001_20260509_143022.bag")
        print("  python3 bag_to_csv.py /data/rosbag/001_20260509_143022.bag ./my_output")
        sys.exit(1)

    bag_file = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else None

    convert_bag_to_csv(bag_file, output_dir)


if __name__ == '__main__':
    main()
