#include "main.h"

namespace display {
/*
static const char * homeBtnMatrixMap[] = {"Auton Selection", "Pre-flight Setup", "\n",
					"Tests", "Graphs", "Calibration", "\n", ""};
lv_obj_t* homeBtnMatrix = lv_btnm_create(lv_scr_act(), NULL);
*/
auto defaultScr = lv_scr_act();

lv_obj_t* autonSelectorBtn = lv_btn_create(lv_scr_act(), NULL);
lv_obj_t* autonSelectorBtnLbl = lv_label_create(autonSelectorBtn, NULL);
lv_obj_t* autonSelectorScr = lv_obj_create(NULL, NULL);

lv_obj_t* preRunBtn = lv_btn_create(lv_scr_act(), NULL);
lv_obj_t* preRunBtnLbl = lv_label_create(preRunBtn, NULL);

lv_obj_t* graphsBtn = lv_btn_create(lv_scr_act(), NULL);
lv_obj_t* graphsBtnLbl = lv_label_create(graphsBtn, NULL);
lv_obj_t* graphsScr = lv_obj_create(NULL, NULL);

lv_obj_t* calibrateBtn = lv_btn_create(lv_scr_act(), NULL);
lv_obj_t* calibrateBtnLbl = lv_label_create(calibrateBtn, NULL);
lv_obj_t* calibrateScr = lv_obj_create(NULL, NULL);

lv_obj_t* homeBtn = lv_btn_create(lv_scr_act(), NULL);
lv_obj_t* homeBtnLbl = lv_label_create(homeBtn, NULL);

lv_res_t autonSelectorBtnAction(lv_obj_t* btn) {
	lv_scr_load(autonSelectorScr);
	lv_obj_set_parent(homeBtn, lv_scr_act());
	lv_obj_set_hidden(homeBtn, false);
	return LV_RES_OK;
}
lv_res_t preRunBtnAction(lv_obj_t* btn) {
	liftControl->setTarget(-17.75);
	liftControl->flipDisable(false);
	return LV_RES_OK;
}

lv_res_t graphsBtnAction(lv_obj_t* btn) {
	lv_scr_load(graphsScr);
	lv_obj_set_parent(homeBtn, lv_scr_act());
	lv_obj_set_hidden(homeBtn, false);
	return LV_RES_OK;
}

lv_res_t calibrateBtnAction(lv_obj_t* btn) {
	lv_scr_load(calibrateScr);
	lv_obj_set_parent(homeBtn, lv_scr_act());
	lv_obj_set_hidden(homeBtn, false);
	return LV_RES_OK;
}

lv_res_t homeBtnAction(lv_obj_t* btn) {
	lv_scr_load(defaultScr);
	lv_obj_set_hidden(homeBtn, true);
	return LV_RES_OK;
}

/*
static void homeBtnMatrixHandler(lv_obj_t* obj, lv_event_t event) {
	if (event = LV_EVENT_PRESSED) {
		switch(lv_btnmatrix_get_active_btn_text(obj)) {
			case "Auton Selection":
				lv_scr_load(autonSelectorScr);
				break;
			case "Pre-flight Setup":
				break;
			case "Tests":
				lv_scr_load(testScr);
				break;
			case "Graphs":
				lv_scr_load(graphsScr);
				break;
			case "Calibration":
				lv_scr_load(calibrateScr);
				break;

		}
	}
}
*/
void startScreen() {
	/*
	lv_btnmatrix_set_map(homeBtnMatrix, homeBtnMatrixMap);
	lv_obj_set_event_cb(homeBtnMatrix, homeBtnMatrixHandler);
	lv_obj_align(homeBtnMatrix, NULL, LV_ALIGN_CENTER, 0, 5);
	*/
	
	lv_obj_set_size(homeBtn, 55, 55);
	lv_obj_set_hidden(homeBtn, true);
	lv_obj_set_pos(homeBtn, 420, 181);
	lv_label_set_text(homeBtnLbl, SYMBOL_HOME);
	lv_btn_set_action(homeBtn, LV_BTN_ACTION_CLICK, homeBtnAction);

	lv_obj_set_size(autonSelectorBtn, 240, 55);
	lv_obj_set_pos(autonSelectorBtn, 5, 5);
	lv_label_set_text(autonSelectorBtnLbl, "Autonomous Selection");
	lv_btn_set_action(autonSelectorBtn, LV_BTN_ACTION_CLICK, autonSelectorBtnAction);
	
	lv_obj_set_size(preRunBtn, 225, 55);
	lv_obj_set_pos(preRunBtn, 250, 5);
	lv_label_set_text(preRunBtnLbl, "Pre-Flight Setup");
	lv_btn_set_action(preRunBtn, LV_BTN_ACTION_CLICK, preRunBtnAction);

	lv_obj_set_size(graphsBtn, 160, 55);
	lv_obj_set_pos(graphsBtn, 5, 125);
	lv_label_set_text(graphsBtnLbl, "Graphs & Diags");
	lv_btn_set_action(graphsBtn, LV_BTN_ACTION_CLICK, graphsBtnAction);

	lv_obj_set_size(calibrateBtn, 160, 55);
	lv_obj_set_pos(calibrateBtn, 5, 185);
	lv_label_set_text(calibrateBtnLbl, "Calibration");
	lv_btn_set_action(calibrateBtn, LV_BTN_ACTION_CLICK, calibrateBtnAction);
	
}

} // namespace display
