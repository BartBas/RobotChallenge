#include "LidarController.h"
#include "crow_all.h"
#include <vector>
#include <mutex>

std::vector<LidarPoint> global_scan;
std::mutex scan_mutex;

int main() {
    LidarController lidar("/dev/ttyUSB0");
    if (!lidar.initialize()) {
        return -1;
    }

    crow::SimpleApp app;

    std::thread lidar_thread([&]() {
        while (true) {
            auto scan = lidar.getLatestScan();
            if (!scan.empty()) {
                std::lock_guard<std::mutex> lock(scan_mutex);
                global_scan = scan;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });

    CROW_ROUTE(app, "/data")([](){
        crow::json::wvalue x;
        std::lock_guard<std::mutex> lock(scan_mutex);
        for (size_t i = 0; i < global_scan.size(); i++) {
            x[i]["a"] = global_scan[i].angle;
            x[i]["r"] = global_scan[i].range;
        }
        return x;
    });

    CROW_ROUTE(app, "/")([](){
        return "<html><body style='background:#111; color:#0f0; font-family:sans-serif; text-align:center;'>"
               "<h2>Pi 5 Lidar Radar</h2>"
               "<canvas id='radar' width='600' height='600' style='border:1px solid #333'></canvas>"
               "<script>"
               "  const canvas = document.getElementById('radar');"
               "  const ctx = canvas.getContext('2d');"
               "  async function update() {"
               "    const r = await fetch('/data'); const data = await r.json();"
               "    ctx.clearRect(0,0,600,600);"
               "    ctx.strokeStyle='#333';"
               "    for(let i=1;i<5;i++){ ctx.beginPath(); ctx.arc(300,300,i*75,0,6.28); ctx.stroke(); }"
               "    ctx.strokeStyle='#444';"
               "    ctx.beginPath(); ctx.moveTo(300,0); ctx.lineTo(300,600); ctx.stroke();"
               "    ctx.beginPath(); ctx.moveTo(0,300); ctx.lineTo(600,300); ctx.stroke();"
               "    data.forEach(p => {"
               "      const rad = (p.a - 90) * Math.PI / 180;"
               "      const x = 300 + Math.cos(rad) * p.r * 100;"
               "      const y = 300 - Math.sin(rad) * p.r * 100;"
               "      ctx.fillStyle = '#0f0'; ctx.fillRect(x-1,y-1,3,3);"
               "    });"
               "    requestAnimationFrame(update);"
               "  } update();"
               "</script></body></html>";
    });

    app.port(8080).multithreaded().run();
    lidar_thread.join();
    return 0;
}
