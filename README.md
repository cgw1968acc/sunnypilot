# tnpbx 發布說明（2026-10-30）

**適用車款**：Toyota **Corolla Altis（TSS2）** 與 **RAV4（TSS2）**，**汽油版與油電版**
**分支**：`cgw1968acc/sunnypilot` 的 `tnpbx`
**安裝網址**：`https://installer.comma.ai/cgw1968acc/tnpbx`

tnpbx 是試用分支，以 sunnypilot 原版為基礎，加上在 Altis 油電版上經過實車調校的 Auto Hold，以及給汽油車的加重版 Auto Hold。歡迎 Altis 和 RAV4 車主試用並回報。

## 車款支援
| 車款 | Auto Hold | 其他改良 |
|---|---|---|
| Corolla Altis TSS2（汽油、油電） | ✅ | ✅ |
| RAV4 2019–2021 TSS2（汽油、油電） | ✅ | ✅ |
| RAV4 2022 以後（雷達 ACC 車款） | ❌ sunnypilot 原版即不支援 | ✅ |
| RAV4 Prime（加密車款） | ❌ | ✅ |

## 主要內容

### 1. Auto Hold 改良（油電版：已在 Altis 油電版實車驗證）
- **用輪速脈衝判斷「真的停住」**：原本車速低於約 0.5 km/h 就顯示為 0，車子還在慢慢滑時 Auto Hold 就可能提早夾住。現在改用輪速脈衝判斷，車子真的停住才會接手。
- **平順接手**：駕駛鬆開煞車時才接手，從駕駛當下踩的力道開始，平順增加到保持力道，不會在腳下突然夾一下。
- **依坡度加大保持力**：平地約 1360 N，4 度坡約 2040 N，8 度坡約 3300 N，最多 4080 N（以上為 Altis 實測值）。坡道上也不會比駕駛當時踩的力道還輕。
- **坡道不輕易放開**：保持中如果車子在坡上微動一下，不會立刻放開讓車溜走。
- **油電判斷更可靠**：改從車上的 CAN 訊號判斷是不是油電車，不再依賴有時查不到的開機查詢。

### 2. Auto Hold 加重版（汽油車：⚠️ 實驗性，尚未實測）
原版 Auto Hold 的保持力道（Altis 上約 1360 N），在汽油車平地上可能擋不住怠速爬行。曾有汽油 RAV4 使用原版時，在平地完全擋不住，車子會一邊發出聲音一邊往前爬。汽油車改成：
- **一開始就用較大的保持力**：-1.5 m/s²（Altis 上約 2040 N），平地也一樣。
- **擋不住就自動加力**：保持中如果偵測到車子往前爬（約 20 公分），在 0.1 秒內加大力道，每次加 0.5 m/s²，最多 -3.0 m/s²（Altis 上約 4080 N）。
- 系統會自動判斷油電或汽油，不需要設定。**油電車的行為完全不受影響。**

### 3. Drive+ Link 修正（台灣 Toyota 車美仕頭機）
- 發動後 sunnypilot 會向車上各電腦查詢韌體資料，這會讓車美仕頭機停止讀取車況，Drive+ Link App 因此不再更新。
- **在 sunnypilot 設定裡固定車型後**，開機就會跳過這個查詢，App 可以正常更新。

### 4. 其他
- 螢幕車速改為和儀表一致。
- 預設開啟「煞車起步平順化」和「sunnypilot 停車控制器」。

## 使用方式
1. 在 sunnypilot 的車輛設定中開啟 **Auto Brake Hold**。
2. **ACC 主開關（MAIN）要打開**，巡航不要啟動，Auto Hold 才會作用。
3. D 檔停住後鬆開煞車，Auto Hold 就會接手；踩油門即解除。
4. 想讓 Drive+ Link 正常更新：在設定中**固定車型**（Altis 選 Corolla TSS2，RAV4 選 RAV4 TSS2）。

## ⚠️ 注意事項
- **汽油車加重版 Auto Hold 尚未在任何汽油車上實測。** 汽油車的煞車系統和油電不同，實際力道、幫浦聲音、能保持多久都可能不同。**前幾次使用時，請把腳放在煞車上準備。**
- **RAV4 尚未實測**：所有力道數字都是在 Altis 上量的。RAV4 車身較重，同樣的設定，實際力道和保持效果可能不同。
- 遇到極陡的坡（約 9 度以上），保持力可能不足，請自行踩住煞車。
- **固定車型的注意事項**：保養時如果更新了 ECU 韌體或更換零件，或把裝置移到另一台車，請先取消固定車型開一趟，讓系統重新查詢，再固定回去。
- 其他會送診斷指令的設備或功能（OBD 升窗器、自動上鎖器、自動上鎖／解鎖功能等），仍會讓 Drive+ Link 停止更新。
- 這是試用版本，請自行評估風險，駕駛人隨時負有完全的駕駛責任。

## 請回報
歡迎回報以下資訊，特別是**汽油車**和 **RAV4**：
- 車款（Altis 或 RAV4）、汽油或油電、年份。
- 日期與時間、地點是平地或坡道。
- Auto Hold 有沒有穩穩擋住、有沒有往前爬、有沒有異常聲音或頓挫。
- 停車時可以按書籤，方便之後查紀錄。
- **最好能附上 comma connect 的 route ID，以供分析確認。**

### 如何提供 route ID
1. 打開 [comma connect](https://connect.comma.ai)，選擇該次行程。
2. 網址列或行程資訊中的 route ID 格式類似：`a1b2c3d4e5f6a7b8/0000012a--3f9e8d7c6b`
3. 為了讓我能讀取完整資料，請在該行程中：
   - **開啟公開（Public）**，或另外把存取權分享給我。
   - **按「Upload all logs」**，上傳完整紀錄（rlog），不只是預覽用的精簡紀錄。建議在連上 Wi-Fi 時上傳，比較快也不耗流量。
   - **按「Preserve」保留**，避免紀錄被自動刪除。
4. 附上發生的大概時間（以手機時間為準），我可以直接找到那次停車。

有了 route ID 和完整紀錄，我就能看到當時的煞車力道、保持指令、車子有沒有移動等數據，確認 Auto Hold 的實際表現。

汽油版和 RAV4 確認正常後，會正式納入後續版本。

## 回到原本的版本
重新安裝原本使用的分支即可，例如 `https://installer.comma.ai/sunnypilot/release`。

---
This software is licensed under a custom license requiring permission for use.
This project uses software from Haibin Wen and SUNNYPILOT LLC and is licensed under a custom license requiring permission for use.
Auto Hold 改良與汽油車加重版：cgw1968acc（由 Claude 協助開發）；顧問：rav4kumar (Kumar)。

---

![](https://user-images.githubusercontent.com/47793918/233812617-beab2e71-57b9-479e-8bff-c3931347ca40.png)

## 🌞 What is sunnypilot?
[sunnypilot](https://github.com/sunnyhaibin/sunnypilot) is a fork of comma.ai's openpilot, an open source driver assistance system. sunnypilot offers the user a unique driving experience for over 300+ supported car makes and models with modified behaviors of driving assist engagements. sunnypilot complies with comma.ai's safety rules as accurately as possible.

## 💭 Join our Community Forum
Join the official sunnypilot community forum to stay up to date with all the latest features and be a part of shaping the future of sunnypilot!
* https://community.sunnypilot.ai/

## Documentation
https://docs.sunnypilot.ai/ is your one stop shop for everything from features to installation to FAQ about the sunnypilot

## 🚘 Running on a dedicated device in a car
First, check out this list of items you'll need to [get started](https://community.sunnypilot.ai/t/getting-started-using-sunnypilot-in-your-supported-car/251).

## Installation
Next, refer to the sunnypilot community forum for [installation instructions](https://community.sunnypilot.ai/t/read-before-installing-sunnypilot/254), as well as a complete list of [Recommended Branch Installations](https://community.sunnypilot.ai/t/recommended-branch-installations/235).

## 🎆 Pull Requests
We welcome both pull requests and issues on GitHub. Bug fixes are encouraged.

Pull requests should be against the most current `master` branch.

## 📊 User Data

By default, sunnypilot uploads the driving data to comma servers. You can also access your data through [comma connect](https://connect.comma.ai/).

sunnypilot is open source software. The user is free to disable data collection if they wish to do so.

sunnypilot logs the road-facing camera, CAN, GPS, IMU, magnetometer, thermal sensors, crashes, and operating system logs.
The driver-facing camera and microphone are only logged if you explicitly opt-in in settings.

By using this software, you understand that use of this software or its related services will generate certain types of user data, which may be logged and stored at the sole discretion of comma. By accepting this agreement, you grant an irrevocable, perpetual, worldwide right to comma for the use of this data.

## Licensing

sunnypilot is released under the [MIT License](LICENSE). This repository includes original work as well as significant portions of code derived from [openpilot by comma.ai](https://github.com/commaai/openpilot), which is also released under the MIT license with additional disclaimers.

The original openpilot license notice, including comma.ai’s indemnification and alpha software disclaimer, is reproduced below as required:

> openpilot is released under the MIT license. Some parts of the software are released under other licenses as specified.
>
> Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.
>
> **THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
> YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
> NO WARRANTY EXPRESSED OR IMPLIED.**

For full license terms, please see the [`LICENSE`](LICENSE) file.

## 💰 Support sunnypilot
If you find any of the features useful, consider becoming a [sponsor on GitHub](https://github.com/sponsors/sunnyhaibin) to support future feature development and improvements.


By becoming a sponsor, you will gain access to exclusive content, early access to new features, and the opportunity to directly influence the project's development.


<h3>GitHub Sponsor</h3>

<a href="https://github.com/sponsors/sunnyhaibin">
  <img src="https://user-images.githubusercontent.com/47793918/244135584-9800acbd-69fd-4b2b-bec9-e5fa2d85c817.png" alt="Become a Sponsor" width="300" style="max-width: 100%; height: auto;">
</a>
<br>

<h3>PayPal</h3>

<a href="https://paypal.me/sunnyhaibin0850" target="_blank">
<img src="https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif" alt="PayPal this" title="PayPal - The safer, easier way to pay online!" border="0" />
</a>
<br></br>

Your continuous love and support are greatly appreciated! Enjoy 🥰

<span>-</span> Jason, Founder of sunnypilot
