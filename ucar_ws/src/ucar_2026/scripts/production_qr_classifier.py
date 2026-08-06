#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""QR-text category classifier backed by the Xunfei Spark OpenAI-compatible
HTTP API, with an offline keyword map as fallback.

The ROS mission is Python 2.  This helper prefers the vehicle's Python 3 but
is kept Python-2-compatible (standard library only) so it can run under either
interpreter.  It exchanges one JSON object per line over stdin/stdout:

    {"command": "classify", "qr_text": "..."}
        -> {"category": "日用品|食品|电子产品|null", "source": "spark|local|none",
            "attempts": n, "model": ..., "raw": ..., "error": ...}
    {"command": "close"} -> {"ok": true, "closed": true}

A missing password file disables the remote call; the local map still works.
"""

from __future__ import print_function

import argparse
import json
import os
import sys
import time

try:
    from urllib.request import Request, urlopen
    from urllib.error import URLError, HTTPError
except ImportError:
    from urllib2 import Request, urlopen, URLError, HTTPError

CATEGORIES = ("日用品", "食品", "电子产品")


def force_utf8_stdio():
    """Make stdin/stdout text-mode UTF-8 regardless of locale settings."""
    try:
        if sys.version_info[0] >= 3:
            for stream in (sys.stdin, sys.stdout):
                reconfigure = getattr(stream, "reconfigure", None)
                if callable(reconfigure):
                    reconfigure(encoding="utf-8")
        else:
            import codecs
            sys.stdin = codecs.getreader("utf-8")(sys.stdin)
            sys.stdout = codecs.getwriter("utf-8")(sys.stdout)
    except Exception:
        pass


force_utf8_stdio()

SYSTEM_PROMPT = (
    "你是一个智能仓库分拣助手。用户会给你一个二维码扫描得到的物品文本，"
    "请判断该物品属于三个加工厂类别中的哪一个。只允许输出三类之一："
    "日用品、食品、电子产品。只输出 JSON，格式为 {\"category\": \"食品\"}。")

# Offline fallback: ordered keyword lists per category.  The first keyword
# contained in the scanned text wins.
LOCAL_CATEGORY_MAP = {
    u"日用品": [
        u"纸巾", u"卫生纸", u"湿巾", u"纸", u"洗衣粉", u"洗衣液", u"洗洁精",
        u"牙膏", u"牙刷", u"洗发水", u"沐浴露", u"沐浴液", u"肥皂", u"香皂",
        u"毛巾", u"拖把", u"扫帚", u"扫把", u"垃圾袋", u"衣架", u"拖鞋",
        u"雨伞", u"杯子", u"水杯", u"碗", u"筷子", u"勺子", u"锅", u"水壶",
        u"脸盆", u"桶", u"刷子", u"棉签", u"创可贴", u"胶带", u"剪刀",
        u"指甲刀", u"梳子", u"镜子", u"挂钩", u"收纳箱", u"保鲜膜", u"保鲜袋",
        u"抹布", u"清洁剂", u"除湿袋", u"樟脑丸", u"杀虫剂", u"蚊香",
        u"花露水", u"防晒霜", u"护肤品", u"洗面奶", u"洗手液", u"洗衣皂",
        u"鞋刷", u"洗衣盆", u"水桶", u"保温杯",
    ],
    u"食品": [
        u"面包", u"牛奶", u"酸奶", u"饼干", u"薯片", u"巧克力", u"糖果",
        u"饮料", u"矿泉水", u"纯净水", u"方便面", u"泡面", u"火腿肠",
        u"罐头", u"果冻", u"咖啡", u"茶叶", u"瓜子", u"花生", u"大米",
        u"面粉", u"食用油", u"酱油", u"醋", u"盐", u"糖", u"鸡蛋",
        u"水果", u"苹果", u"香蕉", u"橙子", u"蔬菜", u"零食", u"蛋糕",
        u"点心", u"月饼", u"粽子", u"辣条", u"可乐", u"雪碧", u"果汁",
        u"奶茶", u"冰淇淋", u"雪糕", u"坚果", u"核桃", u"红枣", u"蜂蜜",
        u"罐头", u"腊肠", u"肉干", u"鱼干", u"海带", u"紫菜", u"调料",
        u"味精", u"鸡精", u"香油", u"料酒", u"蚝油", u"豆瓣酱", u"番茄酱",
    ],
    u"电子产品": [
        u"手机", u"充电器", u"耳机", u"电池", u"数据线", u"插座", u"插排",
        u"灯泡", u"键盘", u"鼠标", u"U盘", u"硬盘", u"路由器", u"摄像头",
        u"音箱", u"音响", u"电视", u"遥控器", u"空调", u"洗衣机", u"电饭煲",
        u"电磁炉", u"微波炉", u"吹风机", u"剃须刀", u"充电宝", u"平板",
        u"电脑", u"笔记本", u"手表", u"手电筒", u"收音机", u"电风扇",
        u"风扇", u"台灯", u"吸尘器", u"加湿器", u"净化器", u"打印机",
        u"扫描仪", u"显示器", u"机顶盒", u"耳机线", u"充电线", u"电池充电器",
        u"智能手表", u"电动牙刷",
    ],
}


def emit(payload):
    sys.stdout.write(json.dumps(payload, ensure_ascii=True) + "\n")
    sys.stdout.flush()


def load_local_map(path):
    """Load a custom keyword map; falls back to the built-in one on error."""
    if not path:
        return LOCAL_CATEGORY_MAP
    try:
        with open(path, "rb") as handle:
            data = json.loads(handle.read().decode("utf-8"))
    except (IOError, OSError, ValueError) as exc:
        emit({"ready": False, "error": "cannot load local map %s: %s" % (
            path, exc)})
        return None
    result = {}
    for category in CATEGORIES:
        entries = data.get(category, [])
        if entries:
            result[category] = [str(entry) for entry in entries]
    return result or LOCAL_CATEGORY_MAP


def classify_locally(local_map, qr_text):
    lowered = qr_text.lower()
    for category in CATEGORIES:
        for keyword in local_map.get(category, []):
            if keyword.lower() in lowered:
                return category
    return None


def parse_json_content(content):
    """Tolerantly extract {"category": ...} from the model's reply."""
    try:
        parsed = json.loads(content)
    except ValueError:
        parsed = None
    if isinstance(parsed, dict) and parsed.get("category") in CATEGORIES:
        return parsed["category"]
    start = content.find("{")
    end = content.rfind("}")
    if 0 <= start < end:
        try:
            parsed = json.loads(content[start:end + 1])
        except ValueError:
            return None
        if isinstance(parsed, dict) and parsed.get("category") in CATEGORIES:
            return parsed["category"]
    for category in CATEGORIES:
        if category in content:
            return category
    return None


def post_json(url, headers, body, timeout):
    request = Request(url, data=body.encode("utf-8"), headers=headers)
    response = urlopen(request, timeout=timeout)
    raw = response.read()
    if isinstance(raw, bytes):
        raw = raw.decode("utf-8", "replace")
    return json.loads(raw)


def classify_with_spark(args, password, qr_text):
    headers = {"Content-Type": "application/json"}
    if password:
        headers["Authorization"] = "Bearer " + password
    body = {
        "model": args.model,
        "messages": [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": qr_text},
        ],
        "temperature": 0.5,
        "stream": False,
        "thinking": {"type": args.thinking},
    }
    last_error = ""
    for attempt in range(1, args.retries + 2):
        try:
            response = post_json(
                args.api_base_url, headers, json.dumps(body), args.timeout)
        except HTTPError as exc:
            last_error = "HTTP %s: %s" % (
                exc.code, _http_error_body(exc))
            if attempt < args.retries + 1:
                time.sleep(0.5 * attempt)
            continue
        except (URLError, ValueError, IOError, OSError) as exc:
            last_error = str(exc)
            if attempt < args.retries + 1:
                time.sleep(0.5 * attempt)
            continue
        code = response.get("code", 0)
        if code != 0:
            last_error = "spark code %s: %s" % (
                code, response.get("message", response))
            if attempt < args.retries + 1:
                time.sleep(0.5 * attempt)
            continue
        if isinstance(response.get("error"), dict):
            last_error = "spark error: %s" % response["error"].get(
                "message", response["error"])
            continue
        choices = response.get("choices") or []
        if not choices:
            last_error = "spark returned no choices"
            continue
        content = choices[0].get("message", {}).get("content", "")
        category = parse_json_content(content)
        if category is not None:
            return category, content, last_error
        last_error = "spark reply had no valid category: %s" % content
    return None, "", last_error


def _http_error_body(exc):
    try:
        raw = exc.read()
        if isinstance(raw, bytes):
            raw = raw.decode("utf-8", "replace")
        return raw[:200]
    except Exception:
        return str(exc)


def read_password(path):
    if not path:
        return ""
    try:
        with open(path, "rb") as handle:
            return handle.read().decode("utf-8", "replace").strip()
    except (IOError, OSError):
        return ""


def resolve_url_result(qr_text, timeout=3.0):
    """If qr_text is an http(s) URL, fetch it and return its JSON 'result'
    field as the classification text.  On any failure the original text is
    kept and a short fetch_error is returned for diagnostics."""
    lowered = (qr_text or "").strip().lower()
    if not (lowered.startswith("http://") or lowered.startswith("https://")):
        return qr_text, ""
    try:
        request = Request(qr_text.strip(), headers={"User-Agent": "ucar-qr"})
        response = urlopen(request, timeout=timeout)
        raw = response.read()
        if isinstance(raw, bytes):
            raw = raw.decode("utf-8", "replace")
        data = json.loads(raw)
    except Exception as exc:
        return qr_text, "url fetch failed: %s" % exc
    code = data.get("code") if isinstance(data, dict) else None
    result = data.get("result") if isinstance(data, dict) else None
    if code != 200 or not result:
        return qr_text, "url json code=%s result=%r" % (code, result)
    return str(result), ""


def classify(args, local_map, password, qr_text):
    attempts = 0
    last_error = ""
    resolved, fetch_error = resolve_url_result(qr_text)
    if fetch_error:
        last_error = fetch_error
    if args.api_base_url and password:
        category, raw, spark_error = classify_with_spark(
            args, password, resolved)
        if spark_error:
            last_error = spark_error
        attempts = args.retries + 1
        if category is not None:
            return {
                "category": category,
                "source": "spark",
                "attempts": attempts,
                "model": args.model,
                "raw": raw,
                "resolved_text": resolved,
                "error": last_error,
            }
    category = classify_locally(local_map, resolved)
    if category is not None:
        return {
            "category": category,
            "source": "local",
            "attempts": attempts,
            "model": args.model,
            "raw": "",
            "resolved_text": resolved,
            "error": last_error,
        }
    return {
        "category": None,
        "source": "none",
        "attempts": attempts,
        "model": args.model,
        "raw": "",
        "resolved_text": resolved,
        "error": last_error or "no remote reply and no local keyword match",
    }


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--api-base-url",
        default="https://spark-api-open.xf-yun.com/x2/chat/completions")
    parser.add_argument("--model", default="spark-x")
    parser.add_argument("--password-file", default="")
    parser.add_argument("--retries", type=int, default=2)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--local-map-file", default="")
    parser.add_argument(
        "--thinking", default="disabled",
        choices=("enabled", "disabled", "auto"),
        help="X2 deep-thinking mode; 'disabled' is fastest for classification")
    return parser.parse_args()


def main():
    args = parse_args()
    args.retries = max(0, args.retries)
    args.timeout = max(0.5, args.timeout)
    local_map = load_local_map(args.local_map_file)
    if local_map is None:
        return 1
    password = read_password(args.password_file)
    emit({
        "ready": True,
        "model": args.model,
        "api_base_url": args.api_base_url,
        "remote_configured": bool(args.api_base_url and password),
        "local_keywords": sum(len(entries) for entries in
                              local_map.values()),
    })
    try:
        for raw_line in sys.stdin:
            try:
                command = json.loads(raw_line)
                name = command.get("command")
                if name == "classify":
                    qr_text = command.get("qr_text", "")
                    emit(classify(args, local_map, password, qr_text))
                elif name == "close":
                    emit({"ok": True, "closed": True})
                    break
                else:
                    emit({"ok": False, "error": "unknown command"})
            except Exception as exc:
                emit({"ok": False, "error": str(exc)})
    finally:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
