import rclpy
from rclpy.serialization import deserialize_message
import importlib
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import csv
import os
from datetime import datetime, timezone

def get_ros2_message_type_from_string(type_string):
    """
    Helper function to dynamically import a ROS 2 message type class using importlib.
    Example type_string: 'std_msgs/msg/Float32'
    """
    try:
        parts = type_string.split('/')
        if len(parts) != 3:
            raise ValueError(f"Type string '{type_string}' is not in the expected format 'package/interface/MessageName'")
        
        package_name = parts[0]
        interface_type = parts[1]
        message_name = parts[2]
        
        module_name_to_import = f"{package_name}.{interface_type}"
        
        module = importlib.import_module(module_name_to_import)
        return getattr(module, message_name)
    except ImportError as e:
        print(f"Error importing module for message type '{type_string}': {e}")
        return None
    except AttributeError as e:
        print(f"Error getting attribute (message class) for '{type_string}': {e}")
        return None
    except ValueError as e:
        print(f"Error with type string format for '{type_string}': {e}")
        return None
    except Exception as e:
        print(f"An unexpected error occurred getting message type for '{type_string}': {e}")
        return None

def get_message_headers_and_extractor(msg_type_name, msg_instance):
    """
    Generates CSV headers and a data extraction function for a given message instance.
    This is a generic version and might need customization for specific complex types.
    """
    headers = ['timestamp_ns']
    fields = []

    if msg_instance is None:
        return headers, lambda msg: []

    if hasattr(msg_instance, 'data') and not hasattr(msg_instance, '__slots__'):
        headers.append('data')
        return headers, lambda msg: [getattr(msg, 'data', None)]

    if hasattr(msg_instance, '__slots__'):
        for slot_name in msg_instance.__slots__:
            clean_slot_name = slot_name.lstrip('_')
            field_value = getattr(msg_instance, slot_name)

            if hasattr(field_value, '__slots__'):
                for sub_slot_name in field_value.__slots__:
                    sub_clean_slot_name = sub_slot_name.lstrip('_')
                    headers.append(f"{clean_slot_name}_{sub_clean_slot_name}")
                    fields.append(lambda msg, s1=slot_name, s2=sub_slot_name: getattr(getattr(msg, s1, None), s2, None))
            elif isinstance(field_value, list):
                 headers.append(f"{clean_slot_name}_list_str")
                 fields.append(lambda msg, s1=slot_name: str(getattr(msg, s1, [])))
            else:
                headers.append(clean_slot_name)
                fields.append(lambda msg, s1=slot_name: getattr(msg, s1, None))
    else: 
        for attr_name in dir(msg_instance):
            if not attr_name.startswith('_') and not callable(getattr(msg_instance, attr_name)):
                headers.append(attr_name)
                fields.append(lambda msg, s1=attr_name: getattr(msg, s1, None))
        if len(headers) == 1: 
            headers.append('raw_message_str') 
            fields.append(lambda msg: str(msg))


    extractor = lambda msg: [field_extractor(msg) for field_extractor in fields]
    return headers, extractor


def export_all_topics_to_csv(bag_directory_path, topics_to_skip=None):
    """
    Exports all found topics from a ROS2 bag to separate CSV files.
    Skips topics defined in topics_to_skip.
    """
    if topics_to_skip is None:
        topics_to_skip = []

    storage_options = StorageOptions(uri=bag_directory_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')

    reader = SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag at '{bag_directory_path}': {e}")
        return

    all_topic_metadata = reader.get_all_topics_and_types()
    if not all_topic_metadata:
        print(f"No topics found in bag: {bag_directory_path}")
        return

    print(f"Found topics in bag '{bag_directory_path}':")
    for meta in all_topic_metadata:
        print(f"  - Name: {meta.name}, Type: {meta.type}")

    csv_files = {}
    csv_writers = {}
    topic_extractors = {}
    loaded_message_types = {}

    normalized_bag_basename = os.path.basename(os.path.normpath(bag_directory_path))
    output_dir = f"{normalized_bag_basename}_csv_export"
    os.makedirs(output_dir, exist_ok=True)
    print(f"Exporting CSVs to directory: {output_dir}")

    for topic_meta in all_topic_metadata:
        topic_name = topic_meta.name
        topic_type_str = topic_meta.type

        if topic_name in topics_to_skip:
            print(f"Skipping topic: {topic_name}")
            continue

        msg_type_class = get_ros2_message_type_from_string(topic_type_str)
        if not msg_type_class:
            print(f"Could not load message type for {topic_name} ({topic_type_str}). Skipping CSV export for this topic.")
            continue
        
        loaded_message_types[topic_name] = msg_type_class
        
        try:
            msg_instance_for_header = msg_type_class()
        except Exception as e:
            print(f"Could not instantiate message {topic_type_str} for header generation on topic {topic_name}: {e}. Using basic header.")
            headers = ['timestamp_ns', 'raw_message_str']
            extractor = lambda msg: [str(msg)] 
        else:
            headers, extractor = get_message_headers_and_extractor(topic_type_str, msg_instance_for_header)

        topic_extractors[topic_name] = extractor
        
        csv_filename = os.path.join(output_dir, topic_name.lstrip('/').replace('/', '_') + '.csv')
        try:
            csv_files[topic_name] = open(csv_filename, 'w', newline='')
            csv_writers[topic_name] = csv.writer(csv_files[topic_name])
            csv_writers[topic_name].writerow(headers)
            print(f"Opened '{csv_filename}' for writing topic '{topic_name}'. Headers: {headers}")
        except IOError as e:
            print(f"Error opening CSV file for {topic_name}: {e}")
            if topic_name in csv_files and csv_files[topic_name]:
                csv_files[topic_name].close()
            if topic_name in csv_writers: del csv_writers[topic_name]
            if topic_name in csv_files: del csv_files[topic_name]


    print("\nStarting message processing and writing to CSVs...")
    processed_counts = {name: 0 for name in csv_writers.keys()}

    while reader.has_next():
        try:
            topic_name, serialized_message, timestamp_ns = reader.read_next()
        except Exception as e:
            print(f"Error reading next message from bag: {e}. Attempting to continue or stop.")
            if "Bag exhausted" in str(e):
                 print("Bag seems to be fully read.")
                 break
            continue


        if topic_name in csv_writers and topic_name in loaded_message_types:
            try:
                msg_type_class = loaded_message_types[topic_name]
                deserialized_msg = deserialize_message(serialized_message, msg_type_class)
                
                extractor_func = topic_extractors.get(topic_name)
                if extractor_func:
                    row_data = extractor_func(deserialized_msg)
                    csv_writers[topic_name].writerow([timestamp_ns] + row_data)
                    processed_counts[topic_name] += 1
                else:
                    csv_writers[topic_name].writerow([timestamp_ns, str(deserialized_msg)])
                    processed_counts[topic_name] += 1


            except Exception as e:
                print(f"Error processing/writing message for topic {topic_name} at {timestamp_ns}: {e}")

    for topic_name_key in list(csv_writers.keys()):
        if topic_name_key in csv_files and csv_files[topic_name_key]:
            try:
                csv_files[topic_name_key].close()
                print(f"Closed CSV for {topic_name_key}. Processed {processed_counts.get(topic_name_key, 0)} messages.")
            except Exception as e:
                print(f"Error closing file for {topic_name_key}: {e}")
        if topic_name_key in csv_writers:
            del csv_writers[topic_name_key]
        if topic_name_key in csv_files:
            del csv_files[topic_name_key]
            
    print("Finished exporting all processable topics to CSV.")

if __name__ == '__main__':
    rclpy.init()
    try:

        script_dir = os.path.dirname(os.path.abspath(__file__))
        bag_directory_path = script_dir

        if not os.path.exists(os.path.join(bag_directory_path, 'metadata.yaml')):
            print(f"Error: 'metadata.yaml' not found in the bag directory '{bag_directory_path}'.")
            print("Please ensure this script is in the directory that CONTAINS the .db3 file AND metadata.yaml,")
            print("or that 'bag_directory_path' correctly points to such a directory.")
        else:
            print(f"Attempting to read bag from directory: {bag_directory_path}")
            topics_to_skip_by_default = [
                '/rosout',
                '/parameter_events',
                '/robot_description',
            ]
            export_all_topics_to_csv(bag_directory_path, topics_to_skip=topics_to_skip_by_default)
    finally:
        rclpy.shutdown()