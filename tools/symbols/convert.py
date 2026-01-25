import click
import glob
import os
import re

@click.group()
def cli():
    pass

def find_closing_paren(text, start_index):
    """Finds the index of the matching closing parenthesis."""
    depth = 0
    in_string = False
    escape = False
    
    for i in range(start_index, len(text)):
        char = text[i]
        
        if in_string:
            if char == '"' and not escape:
                in_string = False
            elif char == '\\' and not escape:
                escape = True
            else:
                escape = False
        else:
            if char == '"':
                in_string = True
            elif char == '(':
                depth += 1
            elif char == ')':
                depth -= 1
                if depth == 0:
                    return i + 1 # Return index AFTER the paren
    return -1


@click.command()
@click.option("-s", "--source", prompt="path to symbol files", help="Symbol files directory", type=click.Path(file_okay=False, dir_okay=True, exists=True))
@click.option("-o", "--out", prompt="path to symbol library", help="Symbol library", type=click.File(mode="w"))
def symbols2library(source, out):
    """Converts symbol files into a .kicad_sym library"""
    fixed_version = "20241209"
    header = f"(kicad_symbol_lib (version {fixed_version}) (generator symbol2library)\n"
    out.write(header)

    # Gather all symbol files
    symbols = glob.glob(os.path.join(source, "*.symbol"))
    derived_symbols = glob.glob(os.path.join(source, "*.derived-symbol"))
    
    for symbol in symbols + derived_symbols:
        with open(symbol, "r") as f:
            for line in f:
                # enforce 2-space indentation for the main file
                out.write("  " + line)
                # Ensure the file ends with a newline
                if not line.endswith("\n"):
                    out.write("\n")
                
    out.write(")\n")


@click.command()
@click.option("-s", "--source", prompt="path to symbol library", help="Symbol library", type=click.File("r"))
@click.option("-o", "--out", prompt="path to symbol files", help="Symbol files directory", type=click.Path(file_okay=False, dir_okay=True, exists=False))
def library2symbols(source, out):
    """Converts a .kicad_sym library into separate symbol files"""
    if not os.path.exists(out):
        os.makedirs(out)
    
    content = source.read()
    cursor = 0
    
    print(f"Scanning {len(content)} bytes...")

    while True:

        # look for the token '(symbol'
        start = content.find("(symbol", cursor)
        if start == -1:
            break
            
        # Check if this is a "real" symbol (not inside a string)
        if start > 0 and content[start-1] not in " \t\n\r(":
            cursor = start + 1
            continue

        # Find the end of this block
        end = find_closing_paren(content, start)
        if end == -1:
            print(f"Error: Unclosed symbol starting at {start}")
            break
            
        block = content[start:end]

        # Extract Name
        match = re.search(r'\(symbol\s+"([^"]+)"', block)
        if match:
            name = match.group(1)
            is_derived = "(extends" in block
            
           
            prefix = ""
            # Look backwards from 'start' to find the newline
            line_start = content.rfind('\n', 0, start)
            if line_start != -1:
                prefix = content[line_start+1:start]
            
            clean_lines = []
            for line in block.split('\n'):
                if line.startswith(prefix):
                    clean_lines.append(line[len(prefix):])
                else:
                    clean_lines.append(line)
            clean_block = "\n".join(clean_lines)

            # Save File
            extension = ".derived-symbol" if is_derived else ".symbol"
            safe_name = name.replace("/", "_").replace(":", "_")
            file_name = os.path.join(out, safe_name + extension)
            
            with open(file_name, "w") as f:
                f.write(clean_block + "\n")
        
        cursor = end

if __name__ == "__main__":
    cli.add_command(symbols2library)
    cli.add_command(library2symbols)
    cli()