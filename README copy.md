# Protobuf to USD converter

Protobuf to USD converter is a Python application for dealing with streamlined conversion from protobuf serialized messages to USD scenes.

## How to use 

run proto_to_usd.py script with the protobuf binary file name as the first argument
```bash
/isaac-sim/python.sh proto_to_usd.py [/directory_to_protobuf_binary_file/file_name]
```
There are several optional arguments:
```bash
--usd-format, either 'usd' or 'usda', by default 'usd'
--out, the output directory, by default 'usd_output/'
--height, the height of usd objects, by default 0.5
```

Example:
```bash
python3 proto_to_usd.py sim_map_test_v1_1.binary --usd-format usda --out custom_usd_output/ --height 0.7
```

## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License

