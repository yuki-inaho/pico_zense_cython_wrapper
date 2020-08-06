import toml
import click
from collections import OrderedDict
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


@click.command()
@click.option('--toml-path', '-t', default="{}/../cfg/camera_parameter.toml".format(SCRIPT_DIR))
def main(toml_path):
    toml_decoder = toml.TomlDecoder(_dict=OrderedDict)
    toml_encoder = toml.TomlEncoder(_dict=OrderedDict)
    toml.TomlEncoder = toml_encoder
    dict_toml = toml.load(open(toml_path), _dict=OrderedDict, decoder=toml_decoder)
    dict_toml["Camera0"]["range2"] = -1
    dict_toml["Camera1"]["range2"] = -1
    with open(toml_path, "w") as f:
        toml.encoder.dump(dict_toml, f)
        print("Done")

if __name__ == "__main__":
    main()