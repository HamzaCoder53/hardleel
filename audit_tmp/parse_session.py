import json

with open(r'C:\Users\Aljamea Nairobi\.claude\projects\C--WINDOWS-system32\e01d35ae-5834-415d-8464-520179db5dcb.jsonl', 'r', encoding='utf-8', errors='ignore') as f:
    for line in f:
        if any(k in line for k in ['track width', 'wheel', 'TB6612', 'Pico firmware', 'motor_bridge.log', 'chassis', '18650']):
            try:
                obj = json.loads(line)
                if obj.get('type') == 'assistant' and 'message' in obj:
                    for c in obj['message'].get('content', []):
                        if c.get('type') == 'text' and any(k in c.get('text', '') for k in ['TB6612', 'track', 'wheel', 'motor']):
                            print("=== ASSISTANT ===")
                            print(c['text'][:500])
                            print("...")
                elif obj.get('type') == 'user':
                    print("=== USER ===", obj.get('message', {}).get('content'))
            except:
                pass
