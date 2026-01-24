from pynput import keyboard
import time

def on_press(key):
    """Handle key press events."""
    try:
        print(f"Key pressed: {key}")
        
        if key == keyboard.Key.delete:
            print("✓ DELETE key detected!")
        elif key == keyboard.Key.backspace:
            print("✓ BACKSPACE key detected!")
        elif key == keyboard.Key.space:
            print("✓ SPACE key detected!")
        elif key == keyboard.Key.enter:
            print("✓ ENTER key detected!")
        elif key == keyboard.Key.esc:
            print("✓ ESC key detected - stopping...")
            return False  # Stop listener
        elif hasattr(key, 'char'):
            print(f"✓ Character key: '{key.char}'")
            
    except AttributeError as e:
        print(f"Special key: {key} - Error: {e}")

def main():
    print("=" * 50)
    print("DELETE KEY TEST")
    print("=" * 50)
    print("Press any key to see what's detected")
    print("Press ESC to exit")
    print("=" * 50)
    
    # Start listening
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()
    
    print("\n✓ Test completed!")

if __name__ == "__main__":
    main()