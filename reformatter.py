import re

def format_python_code(code_string):
    """
    Format Python code with specific spacing rules:
    - 2 blank lines between classes
    - 1 blank line between functions
    - Remove double spaces
    - Remove extra blank lines
    - Preserve indentation
    """
    # Handle double spaces within lines but preserve indentation
    lines = code_string.split('\n')
    for i in range(len(lines)):
        if lines[i].strip():  # If line is not empty
            indent = len(lines[i]) - len(lines[i].lstrip())
            content = re.sub(r'  +', ' ', lines[i][indent:])
            lines[i] = lines[i][:indent] + content
    
    formatted_lines = []
    i = 0
    while i < len(lines):
        current_line = lines[i]
        next_line = lines[i + 1] if i + 1 < len(lines) else ""
        
        # Only add non-empty lines or lines that serve as spacing
        if current_line.strip() or (
            next_line.strip() and (
                next_line.strip().startswith('class ') or 
                next_line.strip().startswith('def ')
            )
        ):
            formatted_lines.append(current_line)
            
            # Add appropriate spacing based on what comes next
            if next_line.strip():
                if next_line.strip().startswith('class '):
                    formatted_lines.extend([''] * 2)  # 2 blank lines before classes
                elif next_line.strip().startswith('def '):
                    formatted_lines.append('')  # 1 blank line before functions
        
        i += 1
    
    # Remove trailing blank lines
    while formatted_lines and not formatted_lines[-1].strip():
        formatted_lines.pop()
    
    return '\n'.join(formatted_lines)

def format_file(input_path, output_path=None):
    """
    Format a Python file according to the spacing rules.
    If output_path is not specified, will overwrite the input file.
    """
    with open(input_path, 'r') as f:
        code = f.read()
    
    formatted_code = format_python_code(code)
    
    output_path = output_path or input_path
    with open(output_path, 'w') as f:
        f.write(formatted_code)
    
    print(f"Formatted code has been written to {output_path}")

# Example usage
if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python formatter.py input_file [output_file]")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None
    
    format_file(input_file, output_file)