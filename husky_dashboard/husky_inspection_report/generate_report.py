import json

# Load JSON data
with open("data.json", "r") as f:
    data = json.load(f)

points = data["points"]

# Calculate summary
total_points = len(points)
inspected = len(points)
detected = sum(1 for p in points if p["status"] == "found")
missing = sum(1 for p in points if p["status"] == "missing")

# Generate table rows
rows_html = ""

for p in points:
    status_class = "found" if p["status"] == "found" else "missing"
    row_class = "missing-row" if p["status"] == "missing" else ""

    rows_html += f"""
    <tr class="{row_class}">
        <td>{p['name']}</td>
        <td><img src="{p['image']}"></td>
        <td class="{status_class}">{p['status'].upper()}</td>
        <td>{p['time']}</td>
    </tr>
    """

# Load template
with open("report_template.html", "r") as f:
    template = f.read()

# Replace placeholders
html_content = template.replace("{{generated_time}}", data["generated_time"])
html_content = html_content.replace("{{total_points}}", str(total_points))
html_content = html_content.replace("{{inspected}}", str(inspected))
html_content = html_content.replace("{{detected}}", str(detected))
html_content = html_content.replace("{{missing}}", str(missing))
html_content = html_content.replace("{{rows}}", rows_html)

# Save final report
with open("report.html", "w") as f:
    f.write(html_content)

print("✅ Report generated: report.html")