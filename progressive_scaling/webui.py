#!/usr/bin/env python3
"""
Script to run the Delta Action Stats Scaler web interface.
Usage: python run_scaler.py [--initial-alpha ALPHA] [--increment INCREMENT]

Arguments:
    --initial-alpha: Initial alpha value (default: 0.1, range: 0.01-1.0)
    --increment: Alpha increment step (default: 0.1, range: 0.01-1.0)

Examples:
    python run_scaler.py
    python run_scaler.py --initial-alpha 0.5
    python run_scaler.py --initial-alpha 0.2 --increment 0.05
"""

import http.server
import socketserver
import webbrowser
import os
import json
import argparse
from pathlib import Path
from urllib.parse import parse_qs

# HTML content
HTML_CONTENT = '''<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Delta Action Stats Scaler</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
            display: flex;
            justify-content: center;
            align-items: center;
        }

        .container {
            background: white;
            border-radius: 16px;
            box-shadow: 0 20px 60px rgba(0, 0, 0, 0.3);
            max-width: 1200px;
            width: 100%;
            padding: 40px;
        }

        h1 {
            color: #333;
            margin-bottom: 10px;
            font-size: 28px;
        }

        .subtitle {
            color: #666;
            margin-bottom: 30px;
            font-size: 14px;
        }

        .alpha-control {
            background: #f8f9fa;
            padding: 20px;
            border-radius: 12px;
            margin-bottom: 30px;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 20px;
        }

        .alpha-label {
            font-weight: 600;
            color: #333;
            font-size: 18px;
        }

        .alpha-value {
            font-size: 32px;
            font-weight: bold;
            color: #667eea;
            min-width: 80px;
            text-align: center;
        }

        .arrow-btn {
            background: #667eea;
            border: none;
            color: white;
            width: 50px;
            height: 50px;
            border-radius: 50%;
            font-size: 24px;
            cursor: pointer;
            transition: all 0.3s;
            display: flex;
            align-items: center;
            justify-content: center;
        }

        .arrow-btn:hover {
            background: #5568d3;
            transform: scale(1.1);
        }

        .arrow-btn:active {
            transform: scale(0.95);
        }

        .stats-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 20px;
            margin-bottom: 30px;
        }

        .stat-card {
            background: #f8f9fa;
            padding: 20px;
            border-radius: 12px;
            border-left: 4px solid #667eea;
        }

        .stat-card.grip {
            border-left-color: #28a745;
        }

        .stat-title {
            font-weight: 600;
            color: #333;
            margin-bottom: 15px;
            font-size: 16px;
        }

        .stat-values {
            display: flex;
            flex-direction: column;
            gap: 10px;
        }

        .stat-row {
            display: flex;
            justify-content: space-between;
            align-items: center;
        }

        .stat-row label {
            color: #666;
            font-size: 14px;
        }

        .stat-row .value {
            font-family: 'Courier New', monospace;
            color: #333;
            font-weight: 600;
        }

        .original {
            color: #667eea;
        }

        .scaled {
            color: #764ba2;
        }

        .divider {
            height: 1px;
            background: #dee2e6;
            margin: 10px 0;
        }

        .export-btn {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            border: none;
            padding: 15px 40px;
            border-radius: 8px;
            font-size: 16px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.3s;
            width: 100%;
            margin-top: 20px;
        }

        .export-btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 10px 20px rgba(102, 126, 234, 0.3);
        }

        .export-btn:active {
            transform: translateY(0);
        }

        .note {
            background: #fff3cd;
            border-left: 4px solid #ffc107;
            padding: 15px;
            border-radius: 8px;
            margin-top: 20px;
            color: #856404;
            font-size: 14px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Delta Action Stats Scaler</h1>
        <p class="subtitle">Scale denormalization constants for RL exploration control</p>

        <div class="alpha-control">
            <button class="arrow-btn" onclick="decreaseAlpha()">▼</button>
            <div>
                <div class="alpha-label">Alpha (α)</div>
                <div class="alpha-value" id="alphaValue">10%</div>
            </div>
            <button class="arrow-btn" onclick="increaseAlpha()">▲</button>
        </div>

        <div class="stats-grid" id="statsGrid"></div>

        <button class="export-btn" onclick="exportScaledJson()">
            📥 Download Scaled JSON
        </button>

        <div class="note">
            <strong>Note:</strong> Grip constants remain unscaled. All other values are multiplied by the selected alpha percentage.
        </div>
    </div>

    <script>
        let originalData = {};
        let alpha = 0.1;
        const INCREMENT = 0.1;

        // Load original data from server on page load
        async function loadOriginalData() {
            try {
                const response = await fetch('/load');
                if (response.ok) {
                    originalData = await response.json();
                    console.log('Original data loaded successfully');
                    updateDisplay();
                } else {
                    const error = await response.json();
                    alert('Error loading original data: ' + error.message);
                    console.error('Error loading data:', error.message);
                }
            } catch (error) {
                alert('Error loading original data: ' + error.message);
                console.error('Error:', error);
            }
        }

        function increaseAlpha() {
            if (alpha < 1.0) {
                alpha = Math.min(1.0, alpha + INCREMENT);
                updateDisplay();
            }
        }

        function decreaseAlpha() {
            if (alpha > INCREMENT) {
                alpha = Math.max(INCREMENT, alpha - INCREMENT);
                updateDisplay();
            }
        }

        function updateDisplay() {
            document.getElementById('alphaValue').textContent = Math.round(alpha * 100) + '%';
            renderStats();
            autoSaveScaledJson();
        }

        function renderStats() {
            const grid = document.getElementById('statsGrid');
            grid.innerHTML = '';

            if (Object.keys(originalData).length === 0) {
                grid.innerHTML = '<p style="text-align: center; color: #666;">Loading data...</p>';
                return;
            }

            for (const [key, values] of Object.entries(originalData)) {
                const isGrip = key === 'Grip';
                const scaledMin = isGrip ? values.min : values.min * alpha;
                const scaledMax = isGrip ? values.max : values.max * alpha;

                const card = document.createElement('div');
                card.className = 'stat-card' + (isGrip ? ' grip' : '');
                card.innerHTML = `
                    <div class="stat-title">${key}</div>
                    <div class="stat-values">
                        <div class="stat-row">
                            <label>Original Min:</label>
                            <span class="value original">${values.min.toFixed(6)}</span>
                        </div>
                        <div class="stat-row">
                            <label>Scaled Min:</label>
                            <span class="value scaled">${scaledMin.toFixed(6)}</span>
                        </div>
                        <div class="divider"></div>
                        <div class="stat-row">
                            <label>Original Max:</label>
                            <span class="value original">${values.max.toFixed(6)}</span>
                        </div>
                        <div class="stat-row">
                            <label>Scaled Max:</label>
                            <span class="value scaled">${scaledMax.toFixed(6)}</span>
                        </div>
                    </div>
                `;
                grid.appendChild(card);
            }
        }

        function exportScaledJson() {
            const scaledData = {};
            
            for (const [key, values] of Object.entries(originalData)) {
                if (key === 'Grip') {
                    scaledData[key] = { ...values };
                } else {
                    scaledData[key] = {
                        min: values.min * alpha,
                        max: values.max * alpha
                    };
                }
            }

            const jsonStr = JSON.stringify(scaledData, null, 4);
            const blob = new Blob([jsonStr], { type: 'application/json' });
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = 'delta_action_stats_teleop_scaled.json';
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
            URL.revokeObjectURL(url);
        }

        function autoSaveScaledJson() {
            const scaledData = {};
            
            for (const [key, values] of Object.entries(originalData)) {
                if (key === 'Grip') {
                    scaledData[key] = { ...values };
                } else {
                    scaledData[key] = {
                        min: values.min * alpha,
                        max: values.max * alpha
                    };
                }
            }

            // Send to server to save
            fetch('/save', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(scaledData)
            })
            .then(response => response.json())
            .then(data => {
                if (data.status === 'success') {
                    console.log('File saved to:', data.path);
                } else {
                    console.error('Error saving file:', data.message);
                }
            })
            .catch(error => {
                console.error('Error:', error);
            });
        }

        // Load data when page loads
        loadOriginalData();
    </script>
</body>
</html>'''

PORT = 8000
SAVE_PATH = "/home/qtf5422/Desktop/AIRE/ibrl-docker/sim_recorder_ee_real/server/data/cube"
file_name = 'delta_action_stats_scaled.json'

# Global variables for alpha settings
INITIAL_ALPHA = 0.1
INCREMENT = 0.1

class MyHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            # Inject the alpha settings into the HTML
            html_with_settings = HTML_CONTENT.replace(
                'let alpha = 0.1;',
                f'let alpha = {INITIAL_ALPHA};'
            ).replace(
                'const INCREMENT = 0.1;',
                f'const INCREMENT = {INCREMENT};'
            )
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(html_with_settings.encode())
        elif self.path == '/load':
            try:
                # Load the original JSON file
                input_path = os.path.join(SAVE_PATH, file_name)
                with open(input_path, 'r') as f:
                    original_data = json.load(f)
                
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(original_data).encode())
                
                print(f"✓ Loaded original data from: {input_path}")
                
            except FileNotFoundError:
                self.send_response(404)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                error_msg = f"File not found: {os.path.join(SAVE_PATH, 'delta_action_stats_teleop.json')}"
                self.wfile.write(json.dumps({'status': 'error', 'message': error_msg}).encode())
                print(f"✗ {error_msg}")
                
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())
                print(f"✗ Error loading file: {e}")
        else:
            super().do_GET()
    
    def do_POST(self):
        if self.path == '/save':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            
            try:
                scaled_data = json.loads(post_data.decode('utf-8'))
                
                # Create directory if it doesn't exist
                os.makedirs(SAVE_PATH, exist_ok=True)
                
                # Save the scaled JSON file
                output_path = os.path.join(SAVE_PATH, 'delta_action_stats_rl.json')
                with open(output_path, 'w') as f:
                    json.dump(scaled_data, f, indent=4)
                
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'success', 'path': output_path}).encode())
                
                print(f"✓ Saved scaled data to: {output_path}")
                
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'error', 'message': str(e)}).encode())
                print(f"✗ Error saving file: {e}")
        else:
            self.send_response(404)
            self.end_headers()

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Run the Delta Action Stats Scaler web interface',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python run_scaler.py
  python run_scaler.py --initial-alpha 0.5
  python run_scaler.py --initial-alpha 0.2 --increment 0.05
  python run_scaler.py --initial-alpha 0.3 --increment 0.01
        """
    )
    parser.add_argument(
        '--initial-alpha',
        type=float,
        default=0.1,
        help='Initial alpha value (default: 0.1, range: 0.01-1.0)'
    )
    parser.add_argument(
        '--increment',
        type=float,
        default=0.1,
        help='Alpha increment step (default: 0.1, range: 0.01-1.0)'
    )
    
    args = parser.parse_args()
    
    # Validate arguments
    if not (0.01 <= args.initial_alpha <= 1.0):
        print("✗ Error: initial-alpha must be between 0.01 and 1.0")
        return
    
    if not (0.01 <= args.increment <= 1.0):
        print("✗ Error: increment must be between 0.01 and 1.0")
        return
    
    # Set global variables
    global INITIAL_ALPHA, INCREMENT
    INITIAL_ALPHA = args.initial_alpha
    INCREMENT = args.increment
    
    print(f"✓ Configuration:")
    print(f"  - Initial Alpha: {INITIAL_ALPHA} ({int(INITIAL_ALPHA * 100)}%)")
    print(f"  - Increment: {INCREMENT} ({int(INCREMENT * 100)}%)")
    print()
    
    # Try to find an available port
    port = PORT
    max_attempts = 10
    
    for attempt in range(max_attempts):
        try:
            with socketserver.TCPServer(("", port), MyHTTPRequestHandler) as httpd:
                url = f"http://localhost:{port}"
                print(f"✓ Server started successfully!")
                print(f"✓ Opening browser at: {url}")
                print(f"✓ Press Ctrl+C to stop the server\n")
                
                # Open browser
                webbrowser.open(url)
                
                # Start serving
                httpd.serve_forever()
                
        except OSError as e:
            if attempt < max_attempts - 1:
                port += 1
                continue
            else:
                print(f"✗ Could not find an available port. Error: {e}")
                return
        except KeyboardInterrupt:
            print("\n\n✓ Server stopped.")
            return

if __name__ == "__main__":
    main()